/*
 * Copyright (c) 2015, Christian Menard
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

#ifndef __MEM_DTU_NOC_ADDR_HH__
#define __MEM_DTU_NOC_ADDR_HH__

#include "base/types.hh"

/**
 *
 *  64    59        52     46          0
 *   -----------------------------------
 *   |res|V| coreId  | epId |  offset  |
 *   -----------------------------------
 */
class NocAddr
{
  public:

    explicit NocAddr() : valid(), coreId(), epId(), offset()
    {}

    explicit NocAddr(Addr addr)
        : valid(addr >> 59),
          coreId((addr >> 52) & ((1 << 7) - 1)),
          epId((addr >> 46) & ((1 << 5) - 1)),
          offset(addr & ((static_cast<Addr>(1) << 46) - 1))
    {}

    explicit NocAddr(unsigned _coreId, unsigned _epId, Addr _offset = 0)
        : valid(1), coreId(_coreId), epId(_epId), offset(_offset)
    {}

    Addr getAddr() const
    {
        assert((coreId & ~((1 << 7) - 1)) == 0);
        assert((epId & ~((1 << 5) - 1)) == 0);
        assert((offset & ~((static_cast<Addr>(1) << 46) - 1)) == 0);

        Addr res = static_cast<Addr>(valid) << 59;
        res |= static_cast<Addr>(coreId) << 52;
        res |= static_cast<Addr>(epId) << 46;
        res |= offset;
        return res;
    }

    bool valid;

    unsigned coreId;

    unsigned epId;

    Addr offset;
};

#endif
