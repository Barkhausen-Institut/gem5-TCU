/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2021 Nils Asmussen, Barkhausen Institut
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

TcuTlb::TcuTlb(Tcu &_tcu, size_t _num)
    : tcu(_tcu), entries(), num(_num), lru_seq()
{
    for (size_t i = 0; i < num; ++i)
        entries.push_back(Entry());
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
TcuTlb::lookup(Addr virt, uint16_t asid, uint access, NocAddr *phys,
               Cycles *delay)
{
    static const char *results[] =
    {
        "HIT",
        "MISS",
        "PAGEFAULT",
    };

    size_t iters;
    Entry *e = do_lookup(virt, asid, &iters);

    *delay = Cycles(tcu.tlbLatency + iters);
    *phys = NocAddr();

    Result res;
    if (!e)
    {
        misses++;
        res = MISS;
    }
    else if ((e->flags & access) != access)
    {
        pagefaults++;
        res = PAGEFAULT;
    }
    else
    {
        assert(e->flags != 0);
        *phys = e->phys;
        e->lru_seq = ++lru_seq;
        Addr mask = (e->flags & LARGE) ? LPAGE_MASK : PAGE_MASK;
        phys->offset += virt & mask;
        hits++;
        res = HIT;
    }

    DPRINTFS(TcuTlbRead, (&tcu),
             "TLB lookup for virt=%p asid=%#x perm=%s -> %s (%p)\n",
             virt, asid, decode_access(access), results[res],
             phys->getAddr());

    return res;
}

TcuTlb::Entry *
TcuTlb::do_lookup(Addr virt, uint16_t asid, size_t *iters)
{
    for(size_t i = 0; i < num; ++i)
    {
        if (entries[i].flags == 0 || entries[i].asid != asid)
            continue;

        Addr offMask = (entries[i].flags & LARGE) ? LPAGE_MASK : PAGE_MASK;
        Addr pgMask = ~offMask;
        if ((virt & pgMask) == entries[i].virt)
        {
            *iters = i + 1;
            return &entries[i];
        }
    }
    *iters = num;
    return nullptr;
}

TcuTlb::Entry *
TcuTlb::find_free()
{
    uint min = std::numeric_limits<uint>::max();
    Entry *minEntry = NULL;
    for (size_t i = 0; i < num; ++i)
    {
        if (entries[i].flags == 0)
            return &entries[i];

        if (entries[i].lru_seq < min && !(entries[i].flags & FIXED))
        {
            min = entries[i].lru_seq;
            minEntry = &entries[i];
        }
    }

    evicts++;
    return minEntry;
}

bool
TcuTlb::insert(Addr virt, uint16_t asid, NocAddr phys, uint flags)
{
    assert(flags != 0);
    Addr mask = (flags & LARGE) ? LPAGE_MASK : PAGE_MASK;

    size_t iters;
    Entry *e = do_lookup(virt, asid, &iters);
    if (!e)
    {
        e = find_free();
        if (!e)
            return false;
    }

    e->asid = asid;
    e->virt = virt & ~mask;
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
    size_t iters;
    Entry *e = do_lookup(virt, asid, &iters);
    if (!e)
        return false;

    DPRINTFS(TcuTlbWrite, (&tcu),
             "TLB invalidate for virt=%p asid=%#x perm=%s -> %p\n",
             virt, asid, decode_access(e->flags), e->phys.getAddr());

    e->flags = 0;
    invalidates++;
    return true;
}

void
TcuTlb::clear()
{
    DPRINTFS(TcuTlbWrite, (&tcu), "TLB flush\n");

    for (size_t i = 0; i < num; ++i)
    {
        if (entries[i].flags != 0 && !(entries[i].flags & FIXED))
            entries[i].flags = 0;
    }
    flushes++;
}
