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

#include "mem/dtu/tlb.hh"
#include "mem/dtu/dtu.hh"

#include "debug/DtuTlb.hh"

#include <limits>

static const char *decode_access(uint access) {
    static char buf[4];
    buf[0] = (access & DtuTlb::INTERN) ? 'i' : '-';
    buf[1] = (access & DtuTlb::READ) ? 'r' : '-';
    buf[2] = (access & DtuTlb::WRITE) ? 'w' : '-';
    buf[3] = (access & DtuTlb::EXEC) ? 'x' : '-';
    return buf;
}

DtuTlb::DtuTlb(Dtu &_dtu, size_t _num)
    : dtu(_dtu), trie(), entries(), free(), num(_num), lru_seq()
{
    for (size_t i = 0; i < num; ++i) {
        entries.push_back(Entry());
        free.push_back(&entries[i]);
    }
}

void
DtuTlb::regStats()
{
    hits
        .name(dtu.name() + ".tlb.hits")
        .desc("Number of TLB accesses that caused a hit");
    misses
        .name(dtu.name() + ".tlb.misses")
        .desc("Number of TLB accesses that caused a miss");
    pagefaults
        .name(dtu.name() + ".tlb.pagefaults")
        .desc("Number of TLB accesses that caused a pagefault");
    noMapping
        .name(dtu.name() + ".tlb.noMapping")
        .desc("Number of TLB accesses that caused an ignored pagefault");
    accesses
        .name(dtu.name() + ".tlb.accesses")
        .desc("Number of total TLB accesses");
    accesses = hits + misses + pagefaults + noMapping;
    inserts
        .name(dtu.name() + ".tlb.inserts")
        .desc("Number of TLB inserts");
    evicts
        .name(dtu.name() + ".tlb.evicts")
        .desc("Number of TLB evictions");
    invalidates
        .name(dtu.name() + ".tlb.invalidates")
        .desc("Number of TLB invalidates");
    flushes
        .name(dtu.name() + ".tlb.flushes")
        .desc("Number of TLB flushes");
}

DtuTlb::Result
DtuTlb::lookup(Addr virt, uint access, NocAddr *phys)
{
    static const char *results[] = {
        "HIT",
        "MISS",
        "PAGEFAULT",
        "NOMAP"
    };

    DtuTlb::Result res = do_lookup(virt, access, phys);

    DPRINTFS(DtuTlbRead, (&dtu), "TLB lookup for %p %s -> %s (%p)\n",
            virt, decode_access(access), results[res], phys->getAddr());

    return res;
}

DtuTlb::Result
DtuTlb::do_lookup(Addr virt, uint access, NocAddr *phys)
{
    Entry *e = trie.lookup(virt);
    if (!e) {
        misses++;
        return MISS;
    }

    if (e->flags == 0) {
        noMapping++;
        return NOMAP;
    }
    // internal accesses to blocked entries pagefault
    // this is only necessary to work around a bug (probably) in the LSQUnit
    if (((access & INTERN) && (e->flags & BLOCKED)) || (e->flags & access) != access) {
        pagefaults++;
        return PAGEFAULT;
    }

    e->lru_seq = ++lru_seq;
    *phys = e->phys;
    phys->offset += virt & PAGE_MASK;
    hits++;
    return HIT;
}

void
DtuTlb::evict()
{
    uint min = std::numeric_limits<uint>::max();
    Entry *minEntry = NULL;
    for (Entry &e : entries)
    {
        if (e.lru_seq < min)
        {
            min = e.lru_seq;
            minEntry = &e;
        }
    }

    assert(minEntry);
    trie.remove(minEntry->handle);
    minEntry->handle = NULL;
    free.push_back(minEntry);
    evicts++;
}

void
DtuTlb::insert(Addr virt, NocAddr phys, uint flags)
{
    Entry *e = trie.lookup(virt);
    if (!e)
    {
        if (free.empty())
            evict();

        assert(!free.empty());
        e = free.back();
        e->virt = virt;
        e->handle = trie.insert(virt, 64 - PAGE_BITS, e);
        free.pop_back();
    }

    DPRINTFS(DtuTlbWrite, (&dtu), "TLB insert for %p %s -> %p\n",
            virt, decode_access(flags), phys.getAddr());

    e->phys = phys;
    e->flags = flags;
    inserts++;
}

void
DtuTlb::block(Addr virt, bool blocked)
{
    Entry *e = trie.lookup(virt);
    if (e)
    {
        DPRINTFS(DtuTlbWrite, (&dtu), "TLB %s %p %s -> %p\n",
                blocked ? "blocking" : "unblocking",
                virt, decode_access(e->flags), e->phys.getAddr());

        if (blocked)
            e->flags |= BLOCKED;
        else
            e->flags &= ~BLOCKED;
    }
}

void
DtuTlb::remove(Addr virt)
{
    Entry *e = trie.lookup(virt);
    if (e)
    {
        DPRINTFS(DtuTlbWrite, (&dtu), "TLB invalidate for %p %s -> %p\n",
                virt, decode_access(e->flags), e->phys.getAddr());

        trie.remove(e->handle);
        e->handle = NULL;
        free.push_back(e);
        invalidates++;
    }
}

void
DtuTlb::clear()
{
    DPRINTFS(DtuTlbWrite, (&dtu), "TLB flush\n");

    for (size_t i = 0; i < num; ++i)
    {
        if (entries[i].handle)
        {
            trie.remove(entries[i].handle);
            entries[i].handle = NULL;
            free.push_back(&entries[i]);
        }
    }
    flushes++;
}
