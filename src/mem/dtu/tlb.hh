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

#include "base/types.hh"
#include "base/trie.hh"
#include "mem/dtu/noc_addr.hh"
#include <vector>

class DtuTlb
{
  private:

    struct Entry
    {
        Addr virt;
        NocAddr phys;
        uint flags;

        uint lru_seq;
        Trie<Addr, Entry>::Handle handle;
    };
    
    typedef Trie<Addr, Entry> TlbEntryTrie;

  public:

    enum
    {
        PAGE_BITS    = 12,
        PAGE_SIZE    = 1 << PAGE_BITS,
    };

    enum Result
    {
        HIT,
        MISS,
        PAGEFAULT,
    };

    // note that these have to match with the bits in PTEs
    enum Flag
    {
        READ    = 1,
        WRITE   = 2,
        EXEC    = 4,
    };

    struct MissHandler
    {
        MissHandler(Addr _virt, Flag _access)
            : virt(_virt), access(_access)
        {}

        virtual ~MissHandler()
        {}

        void start();

        virtual void finish(NocAddr phys) = 0;

        Addr virt;
        Flag access;
    };

    DtuTlb(size_t _num);

    Result lookup(Addr virt, Flag access, NocAddr *phys);

    void insert(Addr virt, NocAddr phys, uint flags);

    void remove(Addr virt);

    void clear();

  private:

    void evict();

    TlbEntryTrie trie;
    std::vector<Entry> entries;
    std::vector<Entry*> free;
    size_t num;
    uint lru_seq;
};

#endif
