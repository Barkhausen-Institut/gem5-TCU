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
#include "debug/DtuPtes.hh"

DTUMemory::DTUMemory(SimObject *obj,
                     uint memPe,
                     Addr memOffset,
                     Addr memSize,
                     PortProxy &phys,
                     uint firstFree,
                     uint vmType)
    : obj(obj),
      physp(phys),
      // don't reuse root pt
      nextFrame(firstFree + 1),
      rootPTOffset(firstFree * DtuTlb::PAGE_SIZE),
      memPe(memPe),
      memOffset(memOffset),
      memSize(memSize),
      vmType(vmType)
{
}

void
DTUMemory::initMemory(System &sys)
{
    // clear root pt
    physp.memsetBlob(getRootPt().getAddr(), 0, DtuTlb::PAGE_SIZE);

    // let the last entry in the root pt point to the root pt itself
    pte_t entry = getRootPt().getAddr() & ~static_cast<pte_t>(DtuTlb::PAGE_MASK);
    // not internally accessible
    entry |= DtuTlb::RWX;
    entry = convertPTE(entry);
    size_t off = 0x10 * sizeof(entry);
    DPRINTFS(DtuPtes, obj,
        "Creating recursive level %d PTE @ %#018x: %#018x\n",
        DtuTlb::LEVEL_CNT - 1, getRootPt().getAddr() + off, entry);
    physp.write(getRootPt().getAddr() + off, entry);

#if THE_ISA == X86_ISA
    if (vmType == CORE)
    {
        // set root PT
        auto ctx = sys.threadContexts[0];
        ctx->setMiscReg(X86ISA::MISCREG_CR3, convertPTE(getRootPt().getAddr()));
    }
#endif
}

void
DTUMemory::mapPage(Addr virt, NocAddr noc, uint access)
{
    Addr ptAddr = getRootPt().getAddr();
    for (int i = DtuTlb::LEVEL_CNT - 1; i >= 0; --i)
    {
        Addr idx = virt >> (DtuTlb::PAGE_BITS + i * DtuTlb::LEVEL_BITS);
        idx &= DtuTlb::LEVEL_MASK;

        Addr pteAddr = ptAddr + (idx << DtuTlb::PTE_BITS);
        pte_t entry = physp.read<pte_t>(pteAddr);
        assert(i > 0 || (entry & DtuTlb::PAGE_MASK) == 0);
        if(!(entry & DtuTlb::PAGE_MASK))
        {
            // determine phys address
            NocAddr addr;
            if (i == 0)
                addr = noc;
            else
            {
                Addr offset = memOffset + (nextFrame++ << DtuTlb::PAGE_BITS);
                addr = NocAddr(memPe, offset);
            }

            // clear pagetables
            if (i > 0)
                physp.memsetBlob(addr.getAddr(), 0, DtuTlb::PAGE_SIZE);

            // insert entry
            entry = addr.getAddr() & ~static_cast<pte_t>(DtuTlb::PAGE_MASK);
            entry |= i == 0 ? static_cast<DtuTlb::Flag>(access) : DtuTlb::IRWX;
            pte_t pte = convertPTE(entry);
            DPRINTFS(DtuPtes, obj,
                "Creating level %d PTE for virt=%#018x @ %#018x: %#018x\n",
                i, virt, pteAddr, pte);
            physp.write(pteAddr, pte);
        }

        ptAddr = entry & ~static_cast<pte_t>(DtuTlb::PAGE_MASK);
    }
}

void
DTUMemory::mapPages(Addr virt, NocAddr noc, Addr size, uint access)
{
    size_t count = divCeil(size, DtuTlb::PAGE_SIZE);
    for(size_t i = 0; i < count; ++i)
    {
        mapPage(virt, noc, access);

        virt += DtuTlb::PAGE_SIZE;
        noc.offset += DtuTlb::PAGE_SIZE;
    }
}

DTUMemory::pte_t
DTUMemory::convertPTE(pte_t pte) const
{
    if(vmType == DTU || pte == 0)
        return pte;

    // the current implementation is based on some equal properties of MMU
    /// and DTU paging
    static_assert(sizeof(pte_t) == 8,
        "MMU and DTU PTEs incompatible");
    static_assert(DtuTlb::LEVEL_CNT == 4,
        "MMU and DTU PTEs incompatible: levels != 4");
    static_assert(DtuTlb::PAGE_SIZE == 4096,
        "MMU and DTU PTEs incompatible: pagesize != 4k");
    static_assert(DtuTlb::LEVEL_BITS == 9,
        "MMU and DTU PTEs incompatible: level bits != 9");

    pte_t res = pte & ~static_cast<pte_t>(DtuTlb::PAGE_MASK);
    // translate NoC address to physical address
    res = (res & ~0xFF00000000000000ULL) | ((res & 0xFF00000000000000ULL) >> 16);

    if(pte & DtuTlb::RWX)
        res |= 0x1;
    if(pte & DtuTlb::WRITE)
        res |= 0x2;
    if(pte & DtuTlb::INTERN)
        res |= 0x4;
    if(pte & DtuTlb::UNCACHE)
        res |= 0x10;
    if(pte & DtuTlb::LARGE)
        res |= 0x80;
    if(~pte & DtuTlb::EXEC)
        res |= 1ULL << 63;
    return res;
}
