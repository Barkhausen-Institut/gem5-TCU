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
 *  64    51        44         0
 *   ---------------------------
 *   |res|V| coreId  | offset  |
 *   ---------------------------
 */
class NocAddr
{
  public:

    explicit NocAddr() : valid(), coreId(), offset()
    {}

    explicit NocAddr(Addr addr)
        : valid(addr >> 51),
          coreId((addr >> 44) & ((1 << 7) - 1)),
          offset(addr & ((static_cast<Addr>(1) << 44) - 1))
    {}

    explicit NocAddr(unsigned _coreId, Addr _offset)
        : valid(1), coreId(_coreId), offset(_offset)
    {}

    Addr getAddr() const
    {
        assert((coreId & ~((1 << 7) - 1)) == 0);
        assert((offset & ~((static_cast<Addr>(1) << 44) - 1)) == 0);

        Addr res = static_cast<Addr>(valid) << 51;
        res |= static_cast<Addr>(coreId) << 44;
        res |= offset;
        return res;
    }

    bool valid;

    unsigned coreId;

    Addr offset;
};

#endif
