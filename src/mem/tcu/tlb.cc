/*
 * Copyright (c) 2015, Nils Asmussen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 */

#include "mem/tcu/tlb.hh"
#include "mem/tcu/tcu.hh"

#include "debug/TcuTlb.hh"

#include <limits>

static const char *decode_access(uint access)
{
    static char buf[6];
    buf[0] = (access & TcuTlb::LARGE) ? 'l' : '-';
    buf[1] = (access & TcuTlb::READ) ? 'r' : '-';
    buf[2] = (access & TcuTlb::WRITE) ? 'w' : '-';
    buf[3] = (access & TcuTlb::EXEC) ? 'x' : '-';
    buf[4] = (access & TcuTlb::FIXED) ? 'f' : '-';
    buf[5] = '\0';
    return buf;
}

static Addr build_key(uint16_t asid, Addr virt)
{
    return (static_cast<Addr>(asid) << 48) | virt;
}

TcuTlb::TcuTlb(Tcu &_tcu, size_t _num)
    : tcu(_tcu), trie(), entries(), free(), num(_num), lru_seq()
{
    for (size_t i = 0; i < num; ++i)
        entries.push_back(Entry());
    for (size_t i = 0; i < num; ++i)
        free.push_back(&entries[i]);
}

void
TcuTlb::regStats()
{
    hits
        .name(tcu.name() + ".tlb.hits")
        .desc("Number of TLB accesses that caused a hit");
    misses
        .name(tcu.name() + ".tlb.misses")
        .desc("Number of TLB accesses that caused a miss");
    pagefaults
        .name(tcu.name() + ".tlb.pagefaults")
        .desc("Number of TLB accesses that caused a pagefault");
    accesses
        .name(tcu.name() + ".tlb.accesses")
        .desc("Number of total TLB accesses");
    accesses = hits + misses + pagefaults;
    inserts
        .name(tcu.name() + ".tlb.inserts")
        .desc("Number of TLB inserts");
    evicts
        .name(tcu.name() + ".tlb.evicts")
        .desc("Number of TLB evictions");
    invalidates
        .name(tcu.name() + ".tlb.invalidates")
        .desc("Number of TLB invalidates");
    flushes
        .name(tcu.name() + ".tlb.flushes")
        .desc("Number of TLB flushes");
}

TcuTlb::Result
TcuTlb::lookup(Addr virt, uint16_t asid, uint access, NocAddr *phys)
{
    static const char *results[] =
    {
        "HIT",
        "MISS",
        "PAGEFAULT",
    };

    TcuTlb::Result res = do_lookup(virt, asid, access, phys);

    DPRINTFS(TcuTlbRead, (&tcu),
             "TLB lookup for virt=%p asid=%#x perm=%s -> %s (%p)\n",
             virt, asid, decode_access(access), results[res], phys->getAddr());

    return res;
}

TcuTlb::Result
TcuTlb::do_lookup(Addr virt, uint16_t asid, uint access, NocAddr *phys)
{
    assert((virt & 0xFFFF000000000000) == 0);
    Entry *e = trie.lookup(build_key(asid, virt));
    if (!e)
    {
        misses++;
        return MISS;
    }

    assert(e->flags != 0);
    if ((e->flags & access) != access)
    {
        pagefaults++;
        return PAGEFAULT;
    }

    e->lru_seq = ++lru_seq;
    *phys = e->phys;
    Addr mask = (e->flags & LARGE) ? LPAGE_MASK : PAGE_MASK;
    phys->offset += virt & mask;
    hits++;
    return HIT;
}

bool
TcuTlb::evict()
{
    uint min = std::numeric_limits<uint>::max();
    Entry *minEntry = NULL;
    for (Entry &e : entries)
    {
        if (e.lru_seq < min && !(e.flags & FIXED))
        {
            min = e.lru_seq;
            minEntry = &e;
        }
    }

    if (!minEntry)
        return false;

    trie.remove(minEntry->handle);
    minEntry->handle = NULL;
    free.push_back(minEntry);
    evicts++;
    return true;
}

bool
TcuTlb::insert(Addr virt, uint16_t asid, NocAddr phys, uint flags)
{
    assert(flags != 0);

    uint width = (flags & LARGE) ? (64 - (PAGE_BITS + LEVEL_BITS))
                                 : (64 - PAGE_BITS);
    Addr mask = (flags & LARGE) ? LPAGE_MASK : PAGE_MASK;

    Addr key = build_key(asid, virt);
    Entry *e = trie.lookup(key);
    if (!e)
    {
        if (free.empty()) {
            if (!evict())
                return false;
        }

        assert(!free.empty());
        e = free.back();
        e->asid = asid;
        e->virt = virt & ~mask;
        e->handle = trie.insert(key, width, e);
        free.pop_back();
    }
    else if((e->flags & LARGE) != (flags & LARGE))
    {
        trie.remove(key);
        e->asid = asid;
        e->virt = virt & ~mask;
        e->handle = trie.insert(key, width, e);
    }

    e->phys = NocAddr(phys.getAddr() & ~mask);
    e->flags = flags;

    DPRINTFS(TcuTlbWrite, (&tcu),
             "TLB insert for virt=%p asid=%#x perm=%s -> %p\n",
             e->virt, e->asid, decode_access(e->flags), e->phys.getAddr());
    inserts++;
    return true;
}

bool
TcuTlb::remove(Addr virt, uint16_t asid)
{
    Entry *e = trie.lookup(build_key(asid, virt));
    if (!e)
        return false;

    DPRINTFS(TcuTlbWrite, (&tcu),
             "TLB invalidate for virt=%p asid=%#x perm=%s -> %p\n",
             virt, asid, decode_access(e->flags), e->phys.getAddr());

    trie.remove(e->handle);
    e->handle = NULL;
    free.push_back(e);
    invalidates++;
    return true;
}

void
TcuTlb::clear()
{
    DPRINTFS(TcuTlbWrite, (&tcu), "TLB flush\n");

    for (size_t i = 0; i < num; ++i)
    {
        if (entries[i].handle && !(entries[i].flags & FIXED))
        {
            trie.remove(entries[i].handle);
            entries[i].handle = NULL;
            free.push_back(&entries[i]);
        }
    }
    flushes++;
}
