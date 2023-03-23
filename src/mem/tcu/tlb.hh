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

#ifndef __MEM_TCU_TLB_HH__
#define __MEM_TCU_TLB_HH__

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/tcu/noc_addr.hh"
#include <vector>

namespace gem5
{
namespace tcu
{

class Tcu;

class TcuTlb
{
  private:

    struct Entry
    {
        Addr virt;
        uint16_t asid;
        NocAddr phys;
        unsigned flags;

        unsigned lru_seq;
    };

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
        FIXED   = 4,
        RW      = READ | WRITE,
    };

    struct MissHandler
    {
        MissHandler(Addr _virt, unsigned _access)
            : virt(_virt), access(_access)
        {}

        virtual ~MissHandler()
        {}

        void start();

        virtual void finish(NocAddr phys) = 0;

        Addr virt;
        unsigned access;
    };

    TcuTlb(Tcu &_tcu, size_t _num);

    void regStats();

    Result lookup(Addr virt, uint16_t asid, unsigned access, NocAddr *phys,
                  Cycles *delay);

    bool insert(Addr virt, uint16_t asid, NocAddr phys, unsigned flags);

    bool remove(Addr virt, uint16_t asid);

    void clear();

  private:

    Entry *do_lookup(Addr virt, uint16_t asid, size_t *iters);

    Entry *find_free();

    Tcu &tcu;
    std::vector<Entry> entries;
    size_t num;
    unsigned lru_seq;

    statistics::Scalar hits;
    statistics::Scalar misses;
    statistics::Scalar pagefaults;
    statistics::Formula accesses;
    statistics::Scalar inserts;
    statistics::Scalar evicts;
    statistics::Scalar invalidates;
    statistics::Scalar flushes;
};

}
}

#endif
