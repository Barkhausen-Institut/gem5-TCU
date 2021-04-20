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

#include "arch/riscv/pagetable_walker.hh"

#include <memory>

#include "arch/riscv/faults.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/tlb.hh"
#include "arch/riscv/vtophys.hh"
#include "base/bitfield.hh"
#include "base/trie.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/PageTableWalker.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"

namespace RiscvISA {

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
    Addr vaddr = req->getVaddr();
    vaddr &= ((static_cast<Addr>(1) << VADDR_BITS) - 1);
    Addr paddr = ourWalker()->tlb->translateWithTLB(vaddr, satp.asid, mode);
    req->setPaddr(paddr);
    delayed = false;
    return NoFault;
}

void
Walker::WalkerState::setupWalk(Addr vaddr)
{
    vaddr &= ((static_cast<Addr>(1) << VADDR_BITS) - 1);

    // fetch these now in case they change during the walk
    status = tc->readMiscReg(MISCREG_STATUS);
    pmode = ourWalker()->tlb->getMemPriv(tc, mode);
    satp = tc->readMiscReg(MISCREG_SATP);
    assert(satp.mode == AddrXlateMode::SV39);

    Addr shift = PageShift + LEVEL_BITS * 2;
    Addr idx = (vaddr >> shift) & LEVEL_MASK;
    Addr topAddr = (satp.ppn << PageShift) + (idx * sizeof(PTESv39));
    level = 2;

    DPRINTF(PageTableWalker, "Performing table walk for address %#x\n", vaddr);
    DPRINTF(PageTableWalker, "Loading level%d PTE from %#x\n", level, topAddr);

    entry.vaddr = vaddr;
    entry.asid = satp.asid;

    Request::Flags flags = Request::PHYSICAL;
    RequestPtr request = std::make_shared<Request>(
        topAddr, sizeof(PTESv39), flags, ourWalker()->masterId);

    read = new Packet(request, MemCmd::ReadReq);
    read->allocate();
}

Fault
Walker::WalkerState::stepWalk(PacketPtr &write)
{
    assert(state != Ready && state != Waiting);
    Fault fault = NoFault;
    write = NULL;
    PTESv39 pte = read->getLE<uint64_t>();
    Addr nextRead = 0;
    bool doWrite = false;
    bool doTLBInsert = false;
    bool doEndWalk = false;

    DPRINTF(PageTableWalker, "Got level%d PTE: %#x\n", level, pte);

    // step 2: TODO check PMA and PMP

    // step 3:
    if (!pte.v || (!pte.r && pte.w)) {
        doEndWalk = true;
        DPRINTF(PageTableWalker, "PTE invalid, raising PF\n");
        fault = pageFault();
    }
    else {
        // step 4:
        if (pte.r || pte.x) {
            // step 5: leaf PTE
            doEndWalk = true;
            fault = ourWalker()->tlb->checkPermissions(status, pmode,
                                                       entry.vaddr, mode, pte);

            // step 6
            if (fault == NoFault) {
                if (level >= 1 && pte.ppn0 != 0) {
                    DPRINTF(PageTableWalker,
                            "PTE has misaligned PPN, raising PF\n");
                    fault = pageFault();
                }
                else if (level == 2 && pte.ppn1 != 0) {
                    DPRINTF(PageTableWalker,
                            "PTE has misaligned PPN, raising PF\n");
                    fault = pageFault();
                }
            }

            if (fault == NoFault) {
                // step 7
                if (!pte.a) {
                    pte.a = 1;
                    doWrite = true;
                }
                if (!pte.d && mode == TLB::Write) {
                    pte.d = 1;
                    doWrite = true;
                }
                // TODO check if this violates a PMA or PMP

                // step 8
                entry.logBytes = PageShift + (level * LEVEL_BITS);
                entry.paddr = pte.ppn;
                entry.vaddr &= ~((1 << entry.logBytes) - 1);
                entry.pte = pte;
                // put it non-writable into the TLB to detect writes and redo
                // the page table walk in order to update the dirty flag.
                if (!pte.d && mode != TLB::Write)
                    entry.pte.w = 0;
                doTLBInsert = true;
            }
        }
        else {
            level--;
            if (level < 0) {
                DPRINTF(PageTableWalker, "No leaf PTE found, raising PF\n");
                doEndWalk = true;
                fault = pageFault();
            }
            else {
                Addr shift = (PageShift + LEVEL_BITS * level);
                Addr idx = (entry.vaddr >> shift) & LEVEL_MASK;
                nextRead = (pte.ppn << PageShift) + (idx * sizeof(pte));
                nextState = Translate;
            }
        }
    }

    PacketPtr oldRead = read;
    Request::Flags flags = oldRead->req->getFlags();

    if (doEndWalk) {
        // If we need to write, adjust the read packet to write the modified
        // value back to memory.
        if (!functional && doWrite) {
            DPRINTF(PageTableWalker, "Writing level%d PTE to %#x: %#x\n",
                level, oldRead->getAddr(), pte);
            write = oldRead;
            write->setLE<uint64_t>(pte);
            write->cmd = MemCmd::WriteReq;
            read = NULL;
        } else {
            write = NULL;
        }

        if (doTLBInsert) {
            if (!functional)
                ourWalker()->tlb->insert(entry.vaddr, entry);
            else {
                DPRINTF(PageTableWalker, "Translated %#x -> %#x\n",
                        entry.vaddr, entry.paddr << PageShift |
                        (entry.vaddr & mask(entry.logBytes)));
            }
        }
        endWalk();
    }
    else {
        //If we didn't return, we're setting up another read.
        RequestPtr request = std::make_shared<Request>(
            nextRead, oldRead->getSize(), flags, ourWalker()->masterId);
        read = new Packet(request, MemCmd::ReadReq);
        read->allocate();

        DPRINTF(PageTableWalker,
                "Loading level%d PTE from %#x\n", level, nextRead);
    }

    return fault;
}

void
Walker::WalkerState::finishFunctional(Addr &addr, unsigned &logBytes)
{
    logBytes = entry.logBytes;
    addr = entry.paddr << PageShift;
}

Fault
Walker::WalkerState::pageFault()
{
    DPRINTF(PageTableWalker, "Raising page fault.\n");
    return ourWalker()->tlb->createPagefault(entry.vaddr, mode);
}

} /* end namespace RiscvISA */

RiscvISA::Walker *
RiscvPagetableWalkerParams::create()
{
    return new RiscvISA::Walker(this);
}
