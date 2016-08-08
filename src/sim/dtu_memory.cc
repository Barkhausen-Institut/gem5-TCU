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

#include "sim/dtu_memory.hh"
#include "mem/port_proxy.hh"
#include "mem/dtu/tlb.hh"
#include "mem/dtu/pt_unit.hh"
#include "debug/DtuTlb.hh"

DTUMemory::DTUMemory(SimObject *obj,
                     unsigned memPe,
                     Addr memOffset,
                     Addr memSize,
                     PortProxy &phys,
                     unsigned firstFree)
    : obj(obj),
      physp(phys),
      // don't reuse root pt
      nextFrame(firstFree),
      memPe(memPe),
      memOffset(memOffset),
      memSize(memSize)
{
}

void
DTUMemory::initMemory()
{
    // clear root pt
    physp.memsetBlob(getRootPt().getAddr(), 0, DtuTlb::PAGE_SIZE);

    // let the last entry in the root pt point to the root pt itself
    PtUnit::PageTableEntry entry = 0;
    entry.base = getRootPt().getAddr() >> DtuTlb::PAGE_BITS;
    // not internally accessible
    entry.ixwr = DtuTlb::RWX;
    size_t off = DtuTlb::PAGE_SIZE - sizeof(entry);
    DPRINTFS(DtuTlb, obj,
        "Creating recursive level %d PTE @ %#018x: %#018x\n",
        DtuTlb::LEVEL_CNT - 1, getRootPt().getAddr() + off, entry);
    physp.write(getRootPt().getAddr() + off, entry);
}

void
DTUMemory::mapPage(Addr virt, Addr phys, uint access)
{
    typedef PtUnit::PageTableEntry pte_t;
    Addr ptAddr = getRootPt().getAddr();
    for (int i = DtuTlb::LEVEL_CNT - 1; i >= 0; --i)
    {
        Addr idx = virt >> (DtuTlb::PAGE_BITS + i * DtuTlb::LEVEL_BITS);
        idx &= DtuTlb::LEVEL_MASK;

        Addr pteAddr = ptAddr + (idx << DtuTlb::PTE_BITS);
        pte_t entry = physp.read<pte_t>(pteAddr);
        assert(i > 0 || entry.ixwr == 0);
        if(!entry.ixwr)
        {
            // determine phys address
            Addr offset;
            if (i == 0)
                offset = memOffset + phys;
            else
                offset = memOffset + (nextFrame++ << DtuTlb::PAGE_BITS);
            NocAddr addr(memPe, offset);

            // clear pagetables
            if (i > 0)
                physp.memsetBlob(addr.getAddr(), 0, DtuTlb::PAGE_SIZE);

            // insert entry
            entry.base = addr.getAddr() >> DtuTlb::PAGE_BITS;
            entry.ixwr = i == 0 ? access : DtuTlb::RWX;
            DPRINTFS(DtuTlb, obj,
                "Creating level %d PTE for virt=%#018x @ %#018x: %#018x\n",
                i, virt, pteAddr, entry);
            physp.write(pteAddr, entry);
        }

        ptAddr = entry.base << DtuTlb::PAGE_BITS;
    }
}

void
DTUMemory::mapSegment(Addr start, Addr size, unsigned perm)
{
    Addr virt = start;
    size_t count = divCeil(size, DtuTlb::PAGE_SIZE);
    for(size_t i = 0; i < count; ++i)
    {
        mapPage(virt, virt, perm);

        virt += DtuTlb::PAGE_SIZE;
    }
}
