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

uint64_t PtUnit::TranslateEvent::nextId = 0;

PtUnit::PtUnit(Dtu& _dtu)
    : dtu(_dtu), lastPfAddr(-1), lastPfCnt(0), translations(), pfqueue()
{}

void
PtUnit::regStats()
{
    walks
        .init(8)
        .name(dtu.name() + ".pt.walks")
        .desc("Page table walk time (in Cycles)")
        .flags(Stats::nozero);
    pagefaults
        .init(8)
        .name(dtu.name() + ".pt.pagefaults")
        .desc("Pagefault time (in Cycles)")
        .flags(Stats::nozero);
    unresolved
        .name(dtu.name() + ".pt.unresolved")
        .desc("Number of unresolved pagefaults");
    delays
        .name(dtu.name() + ".pt.delays")
        .desc("Number of delayed pagefaults due to running pagefaults");
}

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
        finish(false, NocAddr(0));
        return;
    }

    unit.dtu.sendMemRequest(pkt,
                            -1,  // no virtual address here
                            id,
                            Dtu::MemReqType::TRANSLATION,
                            Cycles(0));
}

void
PtUnit::TranslateEvent::process()
{
    if (forceWalk)
    {
        requestPTE();
        forceWalk = false;
        return;
    }

    // first check the TLB again; maybe we don't need to do a translation
    NocAddr phys;
    DtuTlb::Result res = unit.dtu.tlb()->lookup(virt, access, &phys);
    if (res == DtuTlb::HIT)
        finish(true, phys);
    else if (res == DtuTlb::NOMAP)
        finish(false, phys);
    else if (res == DtuTlb::PAGEFAULT)
    {
        if (!unit.sendPagefaultMsg(this, virt, access))
            finish(false, NocAddr(0));
    }
    else
        requestPTE();
}

void
PtUnit::TranslateEvent::recvFromMem(PacketPtr pkt)
{
    Addr phys;
    uint flags = access;
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

        unit.mkTlbEntry(virt, NocAddr(phys), flags);

        finish(success, NocAddr(phys + (virt & DtuTlb::PAGE_MASK)));
    }
    else
    {
        if (!unit.sendPagefaultMsg(this, virt, access))
            finish(false, NocAddr(0));
    }
}

void
PtUnit::TranslateEvent::finish(bool success, const NocAddr &addr)
{
    if (scheduled())
        unit.dtu.deschedule(this);

    if (trans)
        trans->finished(success, addr);
    // make sure that we don't do that twice
    trans->_event = NULL;
    trans = NULL;
    setFlags(AutoDelete);

    auto it = std::find(unit.translations.begin(), unit.translations.end(), this);
    assert(it != unit.translations.end());
    unit.translations.erase(it);

    if (!success)
        unit.resolveFailed(virt);
    unit.nextPagefault(this);

    unit.walks.sample(unit.dtu.curCycle() - startCycle);
}

bool
PtUnit::translateFunctional(Addr virt, uint access, NocAddr *phys)
{
    Addr ptePhys;
    Addr ptAddr = dtu.regs().get(DtuReg::ROOT_PT);
    for (int level = DtuTlb::LEVEL_CNT - 1; level >= 0; --level)
    {
        auto pkt = createPacket(virt, ptAddr, level);
        if (!pkt)
            return false;

        dtu.sendFunctionalMemRequest(pkt);

        uint acccopy = access;
        if (!finishTranslate(pkt, virt, level, &acccopy, &ptePhys))
            return false;

        ptAddr = ptePhys;
    }
    *phys = NocAddr(ptePhys + (virt & DtuTlb::PAGE_MASK));
    return true;
}

const char *
PtUnit::describeAccess(uint access)
{
    static char rwx[4];
    rwx[0] = (access & DtuTlb::READ ) ? 'r' : '-';
    rwx[1] = (access & DtuTlb::WRITE) ? 'w' : '-';
    rwx[2] = (access & DtuTlb::EXEC ) ? 'x' : '-';
    return rwx;
}

bool
PtUnit::sendPagefaultMsg(TranslateEvent *ev, Addr virt, uint access)
{
    if (~dtu.regs().get(DtuReg::FEATURES) & static_cast<int>(Features::PAGEFAULTS))
    {
        DPRINTFS(DtuPf, (&dtu),
            "Pagefault (%llu: %s @ %p), but pagefault sending is disabled\n",
            ev->id, describeAccess(access), virt);

        abortAll();
        return false;
    }

    int pfep = ev->toKernel ? Dtu::SYSCALL_EP : dtu.regs().get(DtuReg::PF_EP);
    assert(pfep < dtu.numEndpoints);
    SendEp ep = dtu.regs().getSendEp(pfep);

    // fall back to syscall EP, if the PF ep is invalid
    if(ep.maxMsgSize == 0)
    {
        ev->toKernel = true;
        pfep = Dtu::SYSCALL_EP;
        ep = dtu.regs().getSendEp(pfep);
    }

    size_t size = sizeof(Dtu::MessageHeader) + sizeof(PagefaultMessage);
    assert(size <= ep.maxMsgSize);

    if (pfqueue.empty())
        pfqueue.push_back(ev);
    else if (pfqueue.front() != ev)
    {
        // try again later
        DPRINTFS(DtuPf, (&dtu),
            "Appending Pagefault (%llu: %s @ %p) to queue\n",
            ev->id, describeAccess(access), virt);

        delays++;
        pfqueue.push_back(ev);
        return true;
    }

    // create packet
    NocAddr nocAddr = NocAddr(ep.targetCore, ep.targetEp);
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
    header.replyLabel = ev->id;
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

    DPRINTFS(Dtu, (&dtu),
             "\e[1m[sd -> %u]\e[0m with EP%u for Pagefault (%llu: %s @ %p)\n",
             ep.targetCore, pfep, ev->id, describeAccess(access), virt);

    DPRINTFS(Dtu, (&dtu),
        "  header: flags=%#x tgtEP=%u lbl=%#018lx rpLbl=%#018lx rpEP=%u\n",
        header.flags, ep.targetEp, header.label,
        header.replyLabel, header.replyEpId);

    // send the packet
    Cycles delay = dtu.transferToNocLatency;
    // delay += dtu.ticksToCycles(headerDelay);
    // pkt->payloadDelay = payloadDelay;
    dtu.printPacket(pkt);
    dtu.sendNocRequest(Dtu::NocPacketType::PAGEFAULT,
                       pkt,
                       ep.vpeId,
                       Dtu::Command::NOPF,
                       delay);

    ev->pfStartCycle = dtu.curCycle();
    return true;
}

void
PtUnit::sendingPfFailed(PacketPtr pkt, int error)
{
    Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();
    uint64_t id = header->label;
    TranslateEvent *ev = getPfEvent(id);

    if (ev)
    {
        DPRINTFS(DtuPf, (&dtu),
            "Sending Pagefault (%llu: %s @ %p) failed (%d); notifying kernel\n",
            id, describeAccess(ev->access), ev->virt, error);

        if (error == static_cast<int>(Dtu::Error::VPE_GONE))
        {
            ev->toKernel = true;
            dtu.schedule(ev, dtu.clockEdge(Cycles(1)));
        }
        else
        {
            panic("Unable to resolve pagefault (%llu: %s @ %p)",
                id, describeAccess(ev->access), ev->virt);
        }
    }
}

void
PtUnit::finishPagefault(PacketPtr pkt)
{
    Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();
    uint64_t *errorPtr = reinterpret_cast<uint64_t*>(header + 1);
    size_t expSize = sizeof(Dtu::MessageHeader) + sizeof(uint64_t);
    int error = pkt->getSize() == expSize ? *errorPtr : -1;

    uint64_t id = header->label;
    TranslateEvent *ev = getPfEvent(id);

    if (!ev)
    {
        DPRINTFS(Dtu, (&dtu),
            "Ignoring unknown (%llu) Pagefault response",
            id);
    }
    else
    {
        DPRINTFS(Dtu, (&dtu),
            "\e[1m[rv <- %u]\e[0m %lu bytes for Pagefault (%llu: %s @ %p)\n",
            header->senderCoreId, header->length,
            id, describeAccess(ev->access), ev->virt);
        dtu.printPacket(pkt);
    }

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

    if (ev)
    {
        pagefaults.sample(dtu.curCycle() - ev->pfStartCycle);

        if (error != 0)
        {
            if (pkt->getSize() != expSize)
                DPRINTFS(DtuPf, (&dtu), "Invalid response for pagefault\n");
            else
            {
                DPRINTFS(DtuPf, (&dtu),
                    "Pagefault (%llu: %s @ %p) could not been resolved: %d\n",
                    id, describeAccess(ev->access), ev->virt, error);
            }

            // if the pagefault handler tells us that there is no mapping, just
            // store an entry with flags=0. this way, we will remember that we
            // already tried to access there with no success
            if (error == static_cast<int>(Dtu::Error::NO_MAPPING))
                mkTlbEntry(ev->virt, NocAddr(0), 0);

            ev->finish(false, NocAddr(0));
            unresolved++;
            return;
        }

        DPRINTFS(DtuPf, (&dtu),
            "Retrying pagetable walk (%llu: %s @ %p)\n",
            id, describeAccess(ev->access), ev->virt);

        // retry the translation
        ev->toKernel = false;
        ev->forceWalk = true;
        dtu.schedule(ev, dtu.clockEdge(Cycles(1)));
    }
}

void PtUnit::mkTlbEntry(Addr virt, NocAddr phys, uint flags)
{
    Addr tlbVirt = virt & ~DtuTlb::PAGE_MASK;

    DPRINTFS(DtuPf, (&dtu),
        "Inserting into TLB: virt=%p phys=%p flags=%u\n",
        tlbVirt, phys.offset, flags);

    dtu.tlb()->insert(tlbVirt, phys, flags);
}

void
PtUnit::nextPagefault(TranslateEvent *ev)
{
    if (!pfqueue.empty() && pfqueue.front() == ev)
    {
        pfqueue.pop_front();

        if (!pfqueue.empty())
        {
            TranslateEvent *ev = pfqueue.front();
            dtu.schedule(ev, dtu.clockEdge(Cycles(1)));
        }
    }
}

PtUnit::TranslateEvent *
PtUnit::getEvent(uint64_t id)
{
    for (auto it = translations.begin(); it != translations.end(); ++it)
    {
        if ((*it)->id == id)
            return *it;
    }
    return nullptr;
}

PtUnit::TranslateEvent *
PtUnit::getPfEvent(uint64_t id)
{
    if (pfqueue.empty() || pfqueue.front()->id != id)
        return NULL;
    return pfqueue.front();
}

PacketPtr
PtUnit::createPacket(Addr virt, Addr ptAddr, int level)
{
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
                        uint *access,
                        Addr *phys)
{
    PageTableEntry *e = pkt->getPtr<PageTableEntry>();

    DPRINTFS(DtuPf, (&dtu), "Received level %d PTE for %p: %#x\n",
             level, virt, (uint64_t)*e);

    // for last-level PTEs, we need the desired access permissions
    if (level == 0)
    {
        if ((e->ixwr & *access) != *access)
            return false;
    }
    // for others, we don't need the INTERN bit
    else
    {
        uint pteAccess = *access & ~DtuTlb::INTERN;
        if ((e->ixwr & pteAccess) != pteAccess)
            return false;
    }

    *access = static_cast<uint>((uint64_t)e->ixwr);
    *phys = e->base << DtuTlb::PAGE_BITS;
    return true;
}

void
PtUnit::resolveFailed(Addr virt)
{
    if (virt == lastPfAddr)
    {
        if (++lastPfCnt == 100)
        {
            dtu.regs().set(DtuReg::LAST_PF, virt);
            // TODO make the vector configurable
            dtu.injectIRQ(0x41);
        }
    }
    else
    {
        lastPfAddr = virt;
        lastPfCnt = 1;
    }
}

void
PtUnit::startTranslate(Addr virt, uint access, Translation *trans)
{
    TranslateEvent *event = new TranslateEvent(*this);
    event->level = DtuTlb::LEVEL_CNT - 1;
    event->virt = virt;
    event->access = access;
    event->trans = trans;
    event->ptAddr = dtu.regs().get(DtuReg::ROOT_PT);
    event->toKernel = false;
    trans->_event = event;
    translations.push_back(event);

    event->startCycle = dtu.curCycle();

    dtu.schedule(event, dtu.clockEdge(Cycles(1)));
}

void
PtUnit::abortTranslate(Translation *trans)
{
    assert(trans->_event != NULL);

    auto it = std::find(pfqueue.begin(), pfqueue.end(), trans->_event);
    if (it != pfqueue.end())
        pfqueue.erase(it);

    trans->_event->finish(false, NocAddr(0));
}

void
PtUnit::abortAll()
{
    if (!pfqueue.empty())
    {
        DPRINTFS(DtuPf, (&dtu),
            "Dropping all pending pagefaults (%lu)\n",
            pfqueue.size());

        while (!pfqueue.empty()) {
            auto qev = pfqueue.front();
            qev->finish(false, NocAddr(0));
        }
    }
}
