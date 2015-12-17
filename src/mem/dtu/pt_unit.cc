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
#include "debug/DtuPf.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/pt_unit.hh"

const std::string
PtUnit::TranslateEvent::name() const
{
    return unit.dtu.name();
}

void
PtUnit::TranslateEvent::requestPTE()
{
    auto pkt = unit.createPacket(virt, ptAddr, level);
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
PtUnit::TranslateEvent::process()
{
    requestPTE();
}

void
PtUnit::TranslateEvent::recvFromMem(PacketPtr pkt)
{
    Addr phys;
    DtuTlb::Flag flags = access;
    bool success = unit.finishTranslate(pkt, virt, level, &flags, &phys);

    if (success)
    {
        if (level > 0)
        {
            level--;
            ptAddr = phys;
            requestPTE();
            return;
        }

        Addr tlbVirt = virt & ~DtuTlb::PAGE_MASK;

        // we can't insert an entry twice
        NocAddr newPhys;
        if (unit.dtu.tlb->lookup(tlbVirt, access, &newPhys) != DtuTlb::HIT)
        {
            DPRINTFS(DtuPf, (&unit.dtu),
                "Inserting into TLB: virt=%p phys=%p flags=%u\n",
                tlbVirt, phys, flags);

            unit.dtu.tlb->insert(tlbVirt, NocAddr(phys), flags);
        }
        else
            assert(newPhys.getAddr() == phys);

        trans->finished(success, NocAddr(phys + (virt & DtuTlb::PAGE_MASK)));

        setFlags(AutoDelete);
    }
    else
        unit.sendPagefaultMsg(this, virt, access);
}

bool
PtUnit::translateFunctional(Addr virt, DtuTlb::Flag access, NocAddr *phys)
{
    Addr ptePhys;
    Addr ptAddr = dtu.regs().get(DtuReg::ROOT_PT);
    for (int level = DtuTlb::LEVEL_CNT - 1; level >= 0; --level)
    {
        auto pkt = createPacket(virt, ptAddr, level);
        if (!pkt)
            return false;

        dtu.sendFunctionalMemRequest(pkt);

        if (!finishTranslate(pkt, virt, level, &access, &ptePhys))
            return false;

        ptAddr = ptePhys;
    }
    *phys = NocAddr(ptePhys + (virt & DtuTlb::PAGE_MASK));
    return true;
}

void
PtUnit::sendPagefaultMsg(TranslateEvent *ev, Addr virt, DtuTlb::Flag access)
{
    if (~dtu.regs().get(DtuReg::STATUS) & static_cast<int>(Status::PAGEFAULTS))
    {
        DPRINTFS(DtuPf, (&dtu),
            "Caused pagefault for %p, but pagefault sending is disabled\n",
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
    int64_t *error = reinterpret_cast<int64_t*>(header + 1);

    TranslateEvent *ev = reinterpret_cast<TranslateEvent*>(header->label);

    size_t expSize = sizeof(Dtu::MessageHeader) + sizeof(int64_t);
    if (pkt->getSize() != expSize || *error != 0)
    {
        if (pkt->getSize() != expSize)
            DPRINTFS(DtuPf, (&dtu), "Invalid response for pagefault\n");
        else
        {
            DPRINTFS(DtuPf, (&dtu),
                "Pagefault for %p could not been resolved: %d\n",
                ev->virt, *(int*)error);
        }
        panic("Pagefault not resolvable; stopping");
    }

    // retry the translation
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
PtUnit::createPacket(Addr virt, Addr ptAddr, int level)
{
    if (virt > dtu.memSize)
        return NULL;

    Addr idx = virt >> (DtuTlb::PAGE_BITS + level * DtuTlb::LEVEL_BITS);
    idx &= DtuTlb::LEVEL_MASK;

    NocAddr pteAddr(ptAddr + (idx << DtuTlb::PTE_BITS));
    auto pkt = dtu.generateRequest(pteAddr.getAddr(),
                                   sizeof(PtUnit::PageTableEntry),
                                   MemCmd::ReadReq);

    DPRINTFS(DtuPf, (&dtu), "Loading level %d PTE for %p from %p\n",
             level, virt, pteAddr.getAddr());

    return pkt;
}

bool
PtUnit::finishTranslate(PacketPtr pkt,
                        Addr virt,
                        int level,
                        DtuTlb::Flag *access,
                        Addr *phys)
{
    PageTableEntry *e = pkt->getPtr<PageTableEntry>();

    DPRINTFS(DtuPf, (&dtu), "Received level %d PTE for %p: %#x\n",
             level, virt, (uint64_t)*e);

    if (!(e->xwr & *access))
        return false;

    *access = static_cast<DtuTlb::Flag>((uint64_t)e->xwr);
    *phys = e->base << DtuTlb::PAGE_BITS;
    return true;
}

void
PtUnit::startTranslate(Addr virt, DtuTlb::Flag access, Translation *trans)
{
    TranslateEvent *event = new TranslateEvent(*this);
    event->level = DtuTlb::LEVEL_CNT - 1;
    event->virt = virt;
    event->access = access;
    event->trans = trans;
    event->ptAddr = dtu.regs().get(DtuReg::ROOT_PT);

    dtu.schedule(event, dtu.clockEdge(Cycles(1)));
}
