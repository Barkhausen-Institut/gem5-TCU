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

static const char *decode_access(uint access)
{
    static char buf[6];
    buf[0] = (access & DtuTlb::INVALID) ? 'I' : '-';
    buf[1] = (access & DtuTlb::INTERN) ? 'i' : '-';
    buf[2] = (access & DtuTlb::READ) ? 'r' : '-';
    buf[3] = (access & DtuTlb::WRITE) ? 'w' : '-';
    buf[4] = (access & DtuTlb::EXEC) ? 'x' : '-';
    buf[5] = '\0';
    return buf;
}

DtuTlb::DtuTlb(Dtu &_dtu, size_t _num)
    : dtu(_dtu), trie(), entries(), free(), num(_num), lru_seq()
{
    for (size_t i = 0; i < num; ++i)
        entries.push_back(Entry());
    for (size_t i = 0; i < num; ++i)
        free.push_back(&entries[i]);
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
DtuTlb::lookup(Addr virt, uint access, NocAddr *phys, bool xlate)
{
    static const char *results[] =
    {
        "HIT",
        "MISS",
        "PAGEFAULT",
        "NOMAP"
    };

    DtuTlb::Result res = do_lookup(virt, access, phys, xlate);

    DPRINTFS(DtuTlbRead, (&dtu), "TLB lookup for %p %s -> %s (%p)\n",
            virt, decode_access(access), results[res], phys->getAddr());

    return res;
}

DtuTlb::Result
DtuTlb::do_lookup(Addr virt, uint access, NocAddr *phys, bool xlate)
{
    Entry *e = trie.lookup(virt);
    if (!e || (e->flags & INVALID))
    {
        misses++;
        return MISS;
    }

    if (e->flags == 0)
    {
        noMapping++;
        return NOMAP;
    }

    if ((e->flags & access) != access)
    {
        pagefaults++;
        return PAGEFAULT;
    }

    // if there is already a translation running, we need to do that as well
    // to maintain ordering
    if (e->xlates > 0 && !xlate)
    {
        misses++;
        return MISS;
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
        if (e.lru_seq < min && e.xlates == 0)
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
        e->xlates = 0;
        free.pop_back();
    }

    e->phys = phys;
    e->flags = flags;

    DPRINTFS(DtuTlbWrite, (&dtu), "TLB insert for %p %s -> %p (%u xlates left)\n",
            virt, decode_access(e->flags), phys.getAddr(), e->xlates);
    inserts++;
}

void
DtuTlb::start_translate(Addr virt)
{
    Entry *e = trie.lookup(virt);
    if (!e)
    {
        insert(virt, NocAddr(0), IRWX | INVALID);
        e = trie.lookup(virt);
    }

    e->xlates++;

    DPRINTFS(DtuTlbWrite, (&dtu), "TLB xlate started for %p (%u xlates left)\n",
            virt, e->xlates);
}

void
DtuTlb::finish_translate(Addr virt)
{
    Entry *e = trie.lookup(virt);
    if (!e)
        return;

    e->xlates--;

    DPRINTFS(DtuTlbWrite, (&dtu), "TLB xlate done for %p (%u xlates left)\n",
            virt, e->xlates);
}

void
DtuTlb::remove(Addr virt)
{
    Entry *e = trie.lookup(virt);
    if (e)
    {
        DPRINTFS(DtuTlbWrite, (&dtu), "TLB invalidate for %p %s -> %p\n",
                virt, decode_access(e->flags), e->phys.getAddr());

        if (e->xlates > 0)
        {
            e->flags |= INVALID;
            return;
        }

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
