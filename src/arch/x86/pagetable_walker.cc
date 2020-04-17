/*
 * Copyright (c) 2012 ARM Limited
 * Copyright (c) 2020 Barkhausen Institut
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/x86/pagetable_walker.hh"

#include <memory>

#include "arch/x86/faults.hh"
#include "arch/x86/pagetable.hh"
#include "arch/x86/tlb.hh"
#include "arch/x86/vtophys.hh"
#include "base/bitfield.hh"
#include "base/trie.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/PageTableWalker.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"

namespace X86ISA {

BaseWalker::BaseWalkerState *
Walker::createState(BaseWalker *walker, BaseTLB::Translation *translation,
                    const RequestPtr &req, bool isFunctional)
{
    return new WalkerState(walker, translation, req, isFunctional);
}

Fault
Walker::WalkerState::translateWithTLB(const RequestPtr &req, ThreadContext *tc,
                                      BaseTLB::Translation *translation,
                                      BaseTLB::Mode mode, bool &delayed)
{
    return ourWalker()->tlb->translate(req, tc, NULL, mode, delayed, true);
}

void
Walker::WalkerState::setupWalk(Addr vaddr)
{
    VAddr addr = vaddr;
    CR3 cr3 = tc->readMiscRegNoEffect(MISCREG_CR3);
    // Check if we're in long mode or not
    Efer efer = tc->readMiscRegNoEffect(MISCREG_EFER);
    dataSize = 8;
    Addr topAddr;
    if (efer.lma) {
        // Do long mode.
        xlState = LongPML4;
        topAddr = (cr3.longPdtb << 12) + addr.longl4 * dataSize;
        enableNX = efer.nxe;
    } else {
        // We're in some flavor of legacy mode.
        CR4 cr4 = tc->readMiscRegNoEffect(MISCREG_CR4);
        if (cr4.pae) {
            // Do legacy PAE.
            xlState = PAEPDP;
            topAddr = (cr3.paePdtb << 5) + addr.pael3 * dataSize;
            enableNX = efer.nxe;
        } else {
            dataSize = 4;
            topAddr = (cr3.pdtb << 12) + addr.norml2 * dataSize;
            if (cr4.pse) {
                // Do legacy PSE.
                xlState = PSEPD;
            } else {
                // Do legacy non PSE.
                xlState = PD;
            }
            enableNX = false;
        }
    }

    entry.vaddr = vaddr;

    Request::Flags flags = Request::PHYSICAL;
    if (cr3.pcd)
        flags.set(Request::UNCACHEABLE);

    RequestPtr request = std::make_shared<Request>(
        topAddr, dataSize, flags, ourWalker()->masterId);

    read = new Packet(request, MemCmd::ReadReq);
    read->allocate();
}

Fault
Walker::WalkerState::stepWalk(PacketPtr &write)
{
    assert(state != Ready && state != Waiting);
    Fault fault = NoFault;
    write = NULL;
    nextXlState = Idle;
    PageTableEntry pte;
    if (dataSize == 8)
        pte = read->getLE<uint64_t>();
    else
        pte = read->getLE<uint32_t>();
    VAddr vaddr = entry.vaddr;
    bool uncacheable = pte.pcd;
    Addr nextRead = 0;
    bool doWrite = false;
    bool doTLBInsert = false;
    bool doEndWalk = false;
    bool badNX = pte.nx && mode == BaseTLB::Execute && enableNX;
    switch(xlState) {
      case LongPML4:
        DPRINTF(PageTableWalker,
                "Got long mode PML4 entry %#016x for %#016x.\n", (uint64_t)pte, vaddr);
        nextRead = ((uint64_t)pte & (mask(40) << 12)) + vaddr.longl3 * dataSize;
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (badNX || !pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        entry.noExec = pte.nx;
        nextXlState = LongPDP;
        break;
      case LongPDP:
        DPRINTF(PageTableWalker,
                "Got long mode PDP entry %#016x for %#016x.\n", (uint64_t)pte, vaddr);
        nextRead = ((uint64_t)pte & (mask(40) << 12)) + vaddr.longl2 * dataSize;
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = entry.writable && pte.w;
        entry.user = entry.user && pte.u;
        if (badNX || !pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        nextXlState = LongPD;
        break;
      case LongPD:
        DPRINTF(PageTableWalker,
                "Got long mode PD entry %#016x for %#016x.\n", (uint64_t)pte, vaddr);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = entry.writable && pte.w;
        entry.user = entry.user && pte.u;
        if (badNX || !pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        if (!pte.ps) {
            // 4 KB page
            entry.logBytes = 12;
            nextRead =
                ((uint64_t)pte & (mask(40) << 12)) + vaddr.longl1 * dataSize;
            nextXlState = LongPTE;
            break;
        } else {
            // 2 MB page
            entry.logBytes = 21;
            entry.paddr = (uint64_t)pte & (mask(31) << 21);
            entry.uncacheable = uncacheable;
            entry.global = pte.g;
            entry.patBit = bits(pte, 12);
            entry.vaddr = entry.vaddr & ~((2 * (1 << 20)) - 1);
            doTLBInsert = true;
            doEndWalk = true;
            break;
        }
      case LongPTE:
        DPRINTF(PageTableWalker,
                "Got long mode PTE entry %#016x for %#016x.\n", (uint64_t)pte, vaddr);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = entry.writable && pte.w;
        entry.user = entry.user && pte.u;
        if (badNX || !pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        entry.paddr = (uint64_t)pte & (mask(40) << 12);
        entry.uncacheable = uncacheable;
        entry.global = pte.g;
        entry.patBit = bits(pte, 12);
        entry.vaddr = entry.vaddr & ~((4 * (1 << 10)) - 1);
        doTLBInsert = true;
        doEndWalk = true;
        break;
      case PAEPDP:
        DPRINTF(PageTableWalker,
                "Got legacy mode PAE PDP entry %#08x for %#08x.\n", (uint32_t)pte, vaddr);
        nextRead = ((uint64_t)pte & (mask(40) << 12)) + vaddr.pael2 * dataSize;
        if (!pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        nextXlState = PAEPD;
        break;
      case PAEPD:
        DPRINTF(PageTableWalker,
                "Got legacy mode PAE PD entry %#08x for %#08x.\n", (uint32_t)pte, vaddr);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (badNX || !pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        if (!pte.ps) {
            // 4 KB page
            entry.logBytes = 12;
            nextRead = ((uint64_t)pte & (mask(40) << 12)) + vaddr.pael1 * dataSize;
            nextXlState = PAEPTE;
            break;
        } else {
            // 2 MB page
            entry.logBytes = 21;
            entry.paddr = (uint64_t)pte & (mask(31) << 21);
            entry.uncacheable = uncacheable;
            entry.global = pte.g;
            entry.patBit = bits(pte, 12);
            entry.vaddr = entry.vaddr & ~((2 * (1 << 20)) - 1);
            doTLBInsert = true;
            doEndWalk = true;
            break;
        }
      case PAEPTE:
        DPRINTF(PageTableWalker,
                "Got legacy mode PAE PTE entry %#08x for %#08x.\n", (uint32_t)pte, vaddr);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = entry.writable && pte.w;
        entry.user = entry.user && pte.u;
        if (badNX || !pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        entry.paddr = (uint64_t)pte & (mask(40) << 12);
        entry.uncacheable = uncacheable;
        entry.global = pte.g;
        entry.patBit = bits(pte, 7);
        entry.vaddr = entry.vaddr & ~((4 * (1 << 10)) - 1);
        doTLBInsert = true;
        doEndWalk = true;
        break;
      case PSEPD:
        DPRINTF(PageTableWalker,
                "Got legacy mode PSE PD entry %#08x for %#08x.\n", (uint32_t)pte, vaddr);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (!pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        if (!pte.ps) {
            // 4 KB page
            entry.logBytes = 12;
            nextRead =
                ((uint64_t)pte & (mask(20) << 12)) + vaddr.norml2 * dataSize;
            nextXlState = PTE;
            break;
        } else {
            // 4 MB page
            entry.logBytes = 21;
            entry.paddr = bits(pte, 20, 13) << 32 | bits(pte, 31, 22) << 22;
            entry.uncacheable = uncacheable;
            entry.global = pte.g;
            entry.patBit = bits(pte, 12);
            entry.vaddr = entry.vaddr & ~((4 * (1 << 20)) - 1);
            doTLBInsert = true;
            doEndWalk = true;
            break;
        }
      case PD:
        DPRINTF(PageTableWalker,
                "Got legacy mode PD entry %#08x for %#08x.\n", (uint32_t)pte, vaddr);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (!pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        // 4 KB page
        entry.logBytes = 12;
        nextRead = ((uint64_t)pte & (mask(20) << 12)) + vaddr.norml2 * dataSize;
        nextXlState = PTE;
        break;
      case PTE:
        DPRINTF(PageTableWalker,
                "Got legacy mode PTE entry %#08x for %#08x.\n", (uint32_t)pte, vaddr);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (!pte.p) {
            doEndWalk = true;
            fault = pageFault(pte.p);
            break;
        }
        entry.paddr = (uint64_t)pte & (mask(20) << 12);
        entry.uncacheable = uncacheable;
        entry.global = pte.g;
        entry.patBit = bits(pte, 7);
        entry.vaddr = entry.vaddr & ~((4 * (1 << 10)) - 1);
        doTLBInsert = true;
        doEndWalk = true;
        break;
      default:
        panic("Unknown page table walker state %d!\n");
    }

    if (nextXlState != Idle)
        nextState = Translate;
    xlState = nextXlState;

    if (doEndWalk) {
        if (doTLBInsert)
            if (!functional)
                ourWalker()->tlb->insert(entry.vaddr, entry);
        endWalk();
    } else {
        PacketPtr oldRead = read;
        //If we didn't return, we're setting up another read.
        Request::Flags flags = oldRead->req->getFlags();
        flags.set(Request::UNCACHEABLE, uncacheable);
        RequestPtr request = std::make_shared<Request>(
            nextRead, oldRead->getSize(), flags, ourWalker()->masterId);
        read = new Packet(request, MemCmd::ReadReq);
        read->allocate();
        // If we need to write, adjust the read packet to write the modified
        // value back to memory.
        if (doWrite) {
            write = oldRead;
            write->setLE<uint64_t>(pte);
            write->cmd = MemCmd::WriteReq;
        } else {
            write = NULL;
            delete oldRead;
        }
    }
    return fault;
}

void
Walker::WalkerState::finishFunctional(Addr &addr, unsigned &logBytes)
{
    logBytes = entry.logBytes;
    addr = entry.paddr;
}

Fault
Walker::WalkerState::pageFault(bool present)
{
    DPRINTF(PageTableWalker, "Raising page fault.\n");
    HandyM5Reg m5reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);
    if (mode == BaseTLB::Execute && !enableNX)
        mode = BaseTLB::Read;
    return std::make_shared<PageFault>(entry.vaddr, present, mode,
                                       m5reg.cpl == 3, false);
}

/* end namespace X86ISA */ }

X86ISA::Walker *
X86PagetableWalkerParams::create()
{
    return new X86ISA::Walker(this);
}
