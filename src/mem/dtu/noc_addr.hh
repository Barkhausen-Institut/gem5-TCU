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
 *  64        56     48          0
 *   -----------------------------
 *   | coreId  | epId |  offset  |
 *   -----------------------------
 */
class NocAddr
{
  public:

    explicit NocAddr() : coreId(), epId(), offset()
    {}

    explicit NocAddr(Addr addr)
        : coreId((addr >> 56) & ((1 << 8) - 1)),
          epId((addr >> 48) & ((1 << 8) - 1)),
          offset(addr & ((static_cast<Addr>(1) << 48) - 1))
    {}

    explicit NocAddr(unsigned _coreId, unsigned _epId, Addr _offset = 0)
        : coreId(_coreId), epId(_epId), offset(_offset)
    {}

    Addr getAddr() const
    {
        assert((coreId & ~((1 << 8) - 1)) == 0);
        assert((epId & ~((1 << 8) - 1)) == 0);
        assert((offset & ~((static_cast<Addr>(1) << 48) - 1)) == 0);

        Addr res = static_cast<Addr>(coreId) << 56;
        res |= static_cast<Addr>(epId) << 48;
        res |= offset;
        return res;
    }

    unsigned coreId;

    unsigned epId;

    Addr offset;
};

#endif
