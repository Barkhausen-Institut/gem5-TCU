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

#include "debug/DtuTlb.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/pt_unit.hh"

const std::string
PtUnit::TranslateEvent::name() const
{
    return unit.dtu.name();
}

void
PtUnit::TranslateEvent::process()
{
    auto pkt = unit.createPacket(virt);
    if (!pkt)
    {
        trans->finished(false, NocAddr(0));
        return;
    }

    unit.dtu.sendMemRequest(pkt,
                            reinterpret_cast<Addr>(this),
                            Dtu::MemReqType::TRANSLATION,
                            Cycles(0));
}

void
PtUnit::TranslateEvent::recvFromMem(PacketPtr pkt)
{
    NocAddr phys;
    DtuTlb::Flag flags = access;
    bool success = unit.finishTranslate(pkt, virt, &flags, &phys);

    if (success)
    {
        Addr tlbVirt = virt & ~DtuTlb::PAGE_MASK;
        Addr tlbPhys = phys.getAddr() & ~DtuTlb::PAGE_MASK;

        // we can't insert an entry twice
        NocAddr newPhys;
        if (unit.dtu.tlb->lookup(tlbVirt, access, &newPhys) != DtuTlb::HIT)
        {
            DPRINTFS(DtuTlb, (&unit.dtu),
                "Inserting into TLB: virt=%p phys=%p flags=%u\n",
                tlbVirt, tlbPhys, flags);

            unit.dtu.tlb->insert(tlbVirt, NocAddr(tlbPhys), flags);
        }
        else
            assert(newPhys.getAddr() == tlbPhys);
    }

    trans->finished(success, phys);

    setFlags(AutoDelete);
}

bool
PtUnit::translateFunctional(Addr virt, DtuTlb::Flag access, NocAddr *phys)
{
    auto pkt = createPacket(virt);
    if (!pkt)
        return false;

    dtu.sendFunctionalMemRequest(pkt);

    return finishTranslate(pkt, virt, &access, phys);
}

PacketPtr
PtUnit::createPacket(Addr virt)
{
    if (virt > dtu.memSize)
        return NULL;

    Addr pageNo = virt / dtu.tlb->PAGE_SIZE;
    Addr pteOff = pageNo * sizeof(PtUnit::PageTableEntry);
    Addr pteAddr = NocAddr(dtu.memPe, 0, dtu.memOffset + pteOff).getAddr();
    auto pkt = dtu.generateRequest(pteAddr,
                                   sizeof(PtUnit::PageTableEntry),
                                   MemCmd::ReadReq);

    DPRINTFS(DtuTlb, (&dtu), "Loading PTE for %p from %p\n",
             virt, pteAddr);

    return pkt;
}

bool
PtUnit::finishTranslate(PacketPtr pkt,
                        Addr virt,
                        DtuTlb::Flag *access,
                        NocAddr *phys)
{
    PtUnit::PageTableEntry *e = pkt->getPtr<PtUnit::PageTableEntry>();

    DPRINTFS(DtuTlb, (&dtu), "Received PTE for %p: %#x\n",
             virt, (uint64_t)*e);

    if (!(e->xwr & *access))
        return false;

    *access = static_cast<DtuTlb::Flag>((uint64_t)e->xwr);
    Addr pageAddr = e->base << DtuTlb::PAGE_BITS;
    *phys = NocAddr(pageAddr + (virt & DtuTlb::PAGE_MASK));
    return true;
}

void
PtUnit::startTranslate(Addr virt, DtuTlb::Flag access, Translation *trans)
{
    TranslateEvent *event = new TranslateEvent(*this);
    event->virt = virt;
    event->access = access;
    event->trans = trans;

    dtu.schedule(event, dtu.clockEdge(Cycles(1)));
}
