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

#include "debug/Dtu.hh"
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

        trans->finished(success, phys);

        setFlags(AutoDelete);
    }
    else
        unit.sendPagefaultMsg(this, virt, access);
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

void
PtUnit::sendPagefaultMsg(TranslateEvent *ev, Addr virt, DtuTlb::Flag access)
{
    if (~dtu.regs().get(DtuReg::STATUS) & static_cast<int>(Status::PAGEFAULTS))
    {
        DPRINTFS(DtuTlb, (&dtu),
            "Caused pagefault for %p,"
            " but pagefault sending is disabled to %u:%u\n",
            virt);
        panic("Pagefault not resolvable; stopping");
    }

    int pfep = dtu.regs().get(DtuReg::PF_EP);
    assert(pfep < dtu.numEndpoints);
    SendEp ep = dtu.regs().getSendEp(pfep);

    // create packet
    NocAddr nocAddr = NocAddr(ep.targetCore, ep.targetEp);
    size_t size = sizeof(Dtu::MessageHeader) + sizeof(PagefaultMessage);
    auto pkt = dtu.generateRequest(nocAddr.getAddr(),
                                   size,
                                   MemCmd::WriteReq);

    // build the message and put it in the packet
    Dtu::MessageHeader header;
    header.length = sizeof(PagefaultMessage);
    header.flags = Dtu::PAGEFAULT | Dtu::REPLY_ENABLED;
    header.label = ep.label;
    header.senderEpId = pfep;
    header.senderCoreId = dtu.coreId;
    header.replyLabel = reinterpret_cast<uint64_t>(ev);
    // not used
    header.replyEpId = 0;

    memcpy(pkt->getPtr<uint8_t>(),
           &header,
           sizeof(header));

    PagefaultMessage msg;
    msg.opcode = PagefaultMessage::OPCODE_PF;
    msg.virt = virt;
    msg.access = access;

    memcpy(pkt->getPtr<uint8_t>() + sizeof(header),
           &msg,
           sizeof(msg));

    DPRINTFS(Dtu, (&dtu), "\e[1m[sd -> %u]\e[0m with EP%u for #PF @ %p\n",
             ep.targetCore, pfep, virt);

    DPRINTFS(Dtu, (&dtu),
        "  header: flags=%#x tgtEP=%u lbl=%#018lx rpLbl=%#018lx rpEP=%u\n",
        header.flags, ep.targetEp, header.label,
        header.replyLabel, header.replyEpId);

    // send the packet
    Cycles delay = dtu.transferToNocLatency;
    // delay += dtu.ticksToCycles(headerDelay);
    // pkt->payloadDelay = payloadDelay;
    dtu.printPacket(pkt);
    dtu.sendNocRequest(Dtu::NocPacketType::MESSAGE, pkt, delay);
}

void
PtUnit::finishPagefault(PacketPtr pkt)
{
    Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

    // retry the translation
    TranslateEvent *ev = reinterpret_cast<TranslateEvent*>(header->label);
    dtu.schedule(ev, dtu.clockEdge(Cycles(1)));

    DPRINTFS(Dtu, (&dtu),
        "\e[1m[rv <- %u]\e[0m %lu bytes for #PF @ %p; retrying\n",
        header->senderCoreId, header->length, ev->virt);
    dtu.printPacket(pkt);

    pkt->makeResponse();

    if (!dtu.atomicMode)
    {
        Cycles delay = dtu.ticksToCycles(
            pkt->headerDelay + pkt->payloadDelay);
        delay += dtu.nocToTransferLatency;

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        dtu.schedNocRequestFinished(dtu.clockEdge(Cycles(1)));
        dtu.schedNocResponse(pkt, dtu.clockEdge(delay));
    }
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
    PageTableEntry *e = pkt->getPtr<PageTableEntry>();

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
