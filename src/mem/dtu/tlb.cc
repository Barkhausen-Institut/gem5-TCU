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

#include <limits>

DtuTlb::DtuTlb(size_t _num, Addr _pageBits)
    : trie(), entries(), free(), num(_num), pageBits(_pageBits), lru_seq()
{
    entries.reserve(num);
    for(size_t i = 0; i < num; ++i)
        free.push_back(&entries[i]);
}

DtuTlb::Result
DtuTlb::lookup(Addr virt, Flag access, NocAddr *phys)
{
    Entry *e = trie.lookup(virt);
    if(!e)
        return MISS;

    if(~e->flags & access)
        return PAGEFAULT;

    e->lru_seq = ++lru_seq;
    *phys = e->phys;
    phys->offset += virt & ((1 << pageBits) - 1);
    return HIT;
}

void
DtuTlb::evict()
{
    uint min = std::numeric_limits<uint>::max();
    Entry *minEntry = NULL;
    for(Entry &e : entries)
    {
        if(e.lru_seq < min)
        {
            min = e.lru_seq;
            minEntry = &e;
        }
    }

    assert(minEntry);
    trie.remove(minEntry->handle);
    minEntry->handle = NULL;
    free.push_back(minEntry);
}

void
DtuTlb::insert(Addr virt, NocAddr phys, uint flags)
{
    if(free.empty())
        evict();

    assert(!free.empty());
    Entry *e = free.back();
    e->virt = virt;
    e->phys = phys;
    e->flags = flags;
    e->handle = trie.insert(virt, 64 - pageBits, e);
    free.pop_back();
}

void
DtuTlb::remove(Addr virt)
{
    Entry *e = trie.lookup(virt);
    if(e)
    {
        trie.remove(e->handle);
        e->handle = NULL;
        free.push_back(e);
    }
}

void
DtuTlb::clear()
{
    for(size_t i = 0; i < num; ++i)
    {
        if(entries[i].handle)
        {
            trie.remove(entries[i].handle);
            entries[i].handle = NULL;
            free.push_back(&entries[i]);
        }
    }
}
