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

#ifndef __MEM_DTU_TLB_HH__
#define __MEM_DTU_TLB_HH__

#include "base/statistics.hh"
#include "base/types.hh"
#include "base/trie.hh"
#include "mem/dtu/noc_addr.hh"
#include <vector>

class Dtu;

class DtuTlb
{
  private:

    struct Entry
    {
        Addr virt;
        NocAddr phys;
        uint flags;
        uint xlates;

        uint lru_seq;
        Trie<Addr, Entry>::Handle handle;
    };

    typedef Trie<Addr, Entry> TlbEntryTrie;

  public:

    enum : Addr
    {
        PTE_BITS     = 3,
        PTE_SIZE     = 1 << PTE_BITS,
        PAGE_BITS    = 12,
        PAGE_SIZE    = 1UL << PAGE_BITS,
        PAGE_MASK    = PAGE_SIZE - 1,
        LEVEL_CNT    = 4,
        LEVEL_BITS   = PAGE_BITS - PTE_BITS,
        LEVEL_MASK   = (1 << LEVEL_BITS) - 1,
        LPAGE_BITS   = PAGE_BITS + LEVEL_BITS,
        LPAGE_SIZE   = 1UL << LPAGE_BITS,
        LPAGE_MASK   = LPAGE_SIZE - 1,
    };

    enum Result
    {
        HIT,
        MISS,
        PAGEFAULT,
        NOMAP
    };

    // note that these have to match with the bits in PTEs
    enum Flag
    {
        READ    = 1,
        WRITE   = 2,
        EXEC    = 4,
        INTERN  = 8,
        LARGE   = 16,
        INVALID = 32,
        RW      = READ | WRITE,
        RX      = READ | EXEC,
        RWX     = RW | EXEC,
        IRWX    = INTERN | RWX
    };

    struct MissHandler
    {
        MissHandler(Addr _virt, uint _access)
            : virt(_virt), access(_access)
        {}

        virtual ~MissHandler()
        {}

        void start();

        virtual void finish(NocAddr phys) = 0;

        Addr virt;
        uint access;
    };

    DtuTlb(Dtu &_dtu, size_t _num);

    void regStats();

    Result lookup(Addr virt, uint access, NocAddr *phys, bool xlate = false);

    void insert(Addr virt, NocAddr phys, uint flags);

    void start_translate(Addr virt);

    void finish_translate(Addr virt);

    void remove(Addr virt);

    void clear();

  private:

    DtuTlb::Result do_lookup(Addr virt, uint access, NocAddr *phys, bool xlate);

    void evict();

    Dtu &dtu;
    TlbEntryTrie trie;
    std::vector<Entry> entries;
    std::vector<Entry*> free;
    size_t num;
    uint lru_seq;

    Stats::Scalar hits;
    Stats::Scalar misses;
    Stats::Scalar pagefaults;
    Stats::Scalar noMapping;
    Stats::Formula accesses;
    Stats::Scalar inserts;
    Stats::Scalar evicts;
    Stats::Scalar invalidates;
    Stats::Scalar flushes;
};

#endif
