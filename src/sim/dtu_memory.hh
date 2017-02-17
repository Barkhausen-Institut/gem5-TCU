/*
 * Copyright (c) 2016, Nils Asmussen
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

#ifndef __SIM_DTU_MEMORY_HH__
#define __SIM_DTU_MEMORY_HH__

#include "sim/system.hh"
#include "mem/dtu/noc_addr.hh"

class DTUMemory
{
  private:

    SimObject *obj;

    PortProxy &physp;
    unsigned nextFrame;

  public:

    const unsigned memPe;
    const Addr memOffset;
    const Addr memSize;

    DTUMemory(SimObject *obj,
              unsigned memPe,
              Addr memOffset,
              Addr memSize,
              PortProxy &phys,
              unsigned firstFree);

    bool hasMem(unsigned pe) const
    {
        return (pedesc(pe) & 0x7) != 1;
    }

    virtual uint32_t pedesc(unsigned pe) const = 0;

    NocAddr getPhys(Addr offset) const
    {
        return NocAddr(memPe, memOffset + offset);
    }

    NocAddr getRootPt() const
    {
        return getPhys(0);
    }

    void initMemory();
    void mapPage(Addr virt, Addr phys, uint access);
    void mapSegment(Addr start, Addr size, unsigned perm);
};

#endif
