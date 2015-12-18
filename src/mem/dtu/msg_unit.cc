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

#include "debug/Dtu.hh"
#include "debug/DtuBuf.hh"
#include "debug/DtuCredits.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuSysCalls.hh"
#include "debug/DtuPower.hh"
#include "debug/DtuTlb.hh"
#include "mem/dtu/msg_unit.hh"
#include "mem/dtu/noc_addr.hh"

static const char *syscallNames[] = {
    "CREATESRV",
    "CREATESESS",
    "CREATEGATE",
    "CREATEVPE",
    "ATTACHRB",
    "DETACHRB",
    "EXCHANGE",
    "VPECTRL",
    "DELEGATE",
    "OBTAIN",
    "ACTIVATE",
    "REQMEM",
    "DERIVEMEM",
    "REVOKE",
    "EXIT",
    "NOOP",
};

void
MessageUnit::startTransmission(const Dtu::Command& cmd)
{
    unsigned epid = cmd.epId;

    // if we want to reply, request the header first
    if (cmd.opcode == Dtu::CommandOpcode::REPLY)
    {
        offset = 0;
        flagsPhys = 0;
        requestHeader(cmd.epId);
        return;
    }

    // check if we have enough credits
    Addr messageSize = dtu.regs().get(CmdReg::DATA_SIZE);
    SendEp ep = dtu.regs().getSendEp(epid);

    // TODO error handling
    assert(messageSize + sizeof(Dtu::MessageHeader) <= ep.maxMsgSize);

    if (ep.credits < ep.maxMsgSize)
    {
        DPRINTFS(Dtu, (&dtu),
            "EP%u: not enough credits (%lu) to send message (%lu)\n",
            epid, ep.credits, ep.maxMsgSize);
        dtu.scheduleFinishOp(Cycles(1));
        return;
    }

    ep.credits -= ep.maxMsgSize;

    DPRINTFS(DtuCredits, (&dtu), "EP%u pays %u credits (%u left)\n",
             epid, ep.maxMsgSize, ep.credits);

    // pay the credits
    dtu.regs().setSendEp(epid, ep);

    // fill the info struct and start the transfer
    info.targetCoreId = ep.targetCore;
    info.targetEpId   = ep.targetEp;
    info.label        = ep.label;
    info.replyLabel   = dtu.regs().get(CmdReg::REPLY_LABEL);
    info.replyEpId    = dtu.regs().get(CmdReg::REPLY_EPID);
    info.flags        = 0;
    info.ready        = true;

    startXfer(cmd);
}

void
MessageUnit::requestHeader(unsigned epid)
{
    assert(offset < sizeof(Dtu::MessageHeader));

    RecvEp ep = dtu.regs().getRecvEp(epid);
    Addr msgAddr = ep.bufAddr + ep.rdOff;
    msgAddr += offset;

    DPRINTFS(DtuBuf, (&dtu),
        "EP%d: requesting header for reply on message @ %p\n",
        epid, msgAddr);

    NocAddr phys(msgAddr);
    if (dtu.tlb)
    {
        uint access = DtuTlb::READ | DtuTlb::INTERN;
        DtuTlb::Result res = dtu.tlb->lookup(msgAddr, access, &phys);
        if (res != DtuTlb::HIT)
        {
            bool pf = res == DtuTlb::PAGEFAULT;
            DPRINTFS(DtuTlb, (&dtu),
                "%s for read access to %p\n",
                pf ? "Pagefault" : "TLB-miss",
                msgAddr);

            Translation *trans = new Translation(*this, epid);
            dtu.startTranslate(msgAddr, DtuTlb::READ, trans, pf);
            return;
        }
    }

    requestHeaderWithPhys(epid, true, phys);
}

void
MessageUnit::requestHeaderWithPhys(unsigned epid,
                                   bool success,
                                   const NocAddr &phys)
{
    // TODO handle error
    assert(success);

    // take care that we might need 2 loads to request the header
    Addr blockOff = (phys.getAddr() + offset) & (dtu.blockSize - 1);
    Addr reqSize = std::min(dtu.blockSize - blockOff,
                            sizeof(Dtu::MessageHeader) - offset);

    auto pkt = dtu.generateRequest(phys.getAddr(),
                                   reqSize,
                                   MemCmd::ReadReq);
    dtu.sendMemRequest(pkt,
                       epid,
                       Dtu::MemReqType::HEADER,
                       Cycles(1));
}

void
MessageUnit::recvFromMem(const Dtu::Command& cmd, PacketPtr pkt)
{
    // simply collect the header in a member for simplicity
    assert(offset + pkt->getSize() <= sizeof(header));
    memcpy(reinterpret_cast<char*>(&header) + offset,
           pkt->getPtr<char*>(),
           pkt->getSize());

    // we need the physical address of the flags field later
    static_assert(offsetof(Dtu::MessageHeader, flags) == 0, "Header changed");
    static_assert(sizeof(header.flags) == 1, "Header changed");
    if (offset == 0)
        flagsPhys = pkt->getAddr();

    offset += pkt->getSize();

    // do we have the complete header yet? if not, request the rest
    if (offset < sizeof(Dtu::MessageHeader))
    {
        requestHeader(cmd.epId);
        return;
    }

    // now that we have the header, fill the info struct
    assert(header.flags & Dtu::REPLY_ENABLED);

    info.targetCoreId = header.senderCoreId;
    info.targetEpId   = header.replyEpId;  // send message to the reply EP
    info.replyEpId    = header.senderEpId; // and grant credits to the sender

    // the receiver of the reply should get the label that he has set
    info.label        = header.replyLabel;
    // replies don't have replies. so, we don't need that
    info.replyLabel   = 0;
    // the pagefault flag is moved to the reply header
    info.flags        = header.flags & Dtu::PAGEFAULT;
    info.ready        = true;

    // disable replies for this message
    // use a functional request here; we don't need to wait for it anyway
    auto hpkt = dtu.generateRequest(flagsPhys,
                                    sizeof(header.flags),
                                    MemCmd::WriteReq);
    header.flags &= ~Dtu::REPLY_ENABLED;
    memcpy(hpkt->getPtr<uint8_t>(), &header.flags, sizeof(header.flags));
    dtu.sendFunctionalMemRequest(hpkt);
    dtu.freeRequest(hpkt);

    // now start the transfer
    startXfer(cmd);
}

void
MessageUnit::startXfer(const Dtu::Command& cmd)
{
    assert(info.ready);

    Addr messageAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr messageSize = dtu.regs().get(CmdReg::DATA_SIZE);

    DPRINTFS(Dtu, (&dtu), "\e[1m[%s -> %u]\e[0m with EP%u of %#018lx:%lu\n",
             cmd.opcode == Dtu::CommandOpcode::REPLY ? "rp" : "sd",
             info.targetCoreId,
             cmd.epId,
             dtu.regs().get(CmdReg::DATA_ADDR),
             messageSize);

    DPRINTFS(Dtu, (&dtu),
        "  header: flags=%#x tgtEP=%u lbl=%#018lx rpLbl=%#018lx rpEP=%u\n",
        info.flags, info.targetEpId, info.label,
        info.replyLabel, info.replyEpId);

    Dtu::MessageHeader* header = new Dtu::MessageHeader;

    if (cmd.opcode == Dtu::CommandOpcode::REPLY)
        header->flags = Dtu::REPLY_FLAG | Dtu::GRANT_CREDITS_FLAG;
    else
        header->flags = Dtu::REPLY_ENABLED; // normal message
    header->flags |= info.flags;

    header->senderCoreId = static_cast<uint8_t>(dtu.coreId);
    header->senderEpId   = static_cast<uint8_t>(cmd.epId);
    header->replyEpId    = static_cast<uint8_t>(info.replyEpId);
    header->length       = static_cast<uint16_t>(messageSize);
    header->label        = static_cast<uint64_t>(info.label);
    header->replyLabel   = static_cast<uint64_t>(info.replyLabel);

    assert(messageSize + sizeof(Dtu::MessageHeader) <= dtu.maxNocPacketSize);

    // start the transfer of the payload
    dtu.startTransfer(Dtu::TransferType::LOCAL_READ,
                      NocAddr(info.targetCoreId, info.targetEpId),
                      messageAddr,
                      messageSize,
                      NULL,
                      header,
                      dtu.startMsgTransferDelay);

    info.ready = false;
}

void
MessageUnit::incrementReadPtr(unsigned epId)
{
    RecvEp ep = dtu.regs().getRecvEp(epId);

    ep.rdOff += ep.msgSize;

    if (ep.rdOff >= ep.size * ep.msgSize)
        ep.rdOff = 0;

    DPRINTFS(DtuBuf, (&dtu),
        "EP%u: increment read pointer to %#018lx (msgCount=%u)\n",
        epId, ep.rdOff, ep.msgCount - 1);

    // TODO error handling
    assert(ep.msgCount != 0);
    ep.msgCount--;

    dtu.regs().setRecvEp(epId, ep);

    dtu.updateSuspendablePin();
}

bool
MessageUnit::incrementWritePtr(unsigned epId)
{
    RecvEp ep = dtu.regs().getRecvEp(epId);

    ep.wrOff += ep.msgSize;

    if (ep.wrOff >= ep.size * ep.msgSize)
        ep.wrOff = 0;

    DPRINTFS(DtuBuf, (&dtu),
        "EP%u: increment write pointer to %#018lx (msgCount=%u)\n",
        epId, ep.wrOff, ep.msgCount + 1);

    if (ep.msgCount == ep.size)
    {
        warn("EP%u: Buffer full!\n", epId);
        return false;
    }
    ep.msgCount++;

    dtu.regs().setRecvEp(epId, ep);

    dtu.updateSuspendablePin();
    dtu.wakeupCore();
    return true;
}

void
MessageUnit::recvFromNoc(PacketPtr pkt)
{
    assert(pkt->isWrite());
    assert(pkt->hasData());

    Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

    uint8_t pfResp = Dtu::REPLY_FLAG | Dtu::PAGEFAULT;
    if ((header->flags & pfResp) == pfResp)
    {
        dtu.handlePFResp(pkt);
        return;
    }

    NocAddr addr(pkt->getAddr());
    unsigned epId = addr.epId;
    RecvEp ep = dtu.regs().getRecvEp(epId);
    Addr localAddr = ep.bufAddr + ep.wrOff;

    DPRINTFS(Dtu, (&dtu),
        "\e[1m[rv <- %u]\e[0m %lu bytes on EP%u to %#018lx\n",
        header->senderCoreId, header->length, epId, localAddr);
    dtu.printPacket(pkt);

    if (dtu.coreId == 0 && epId == 0)
    {
        const size_t total = (sizeof(syscallNames) / sizeof(syscallNames[0]));
        size_t sysNo = pkt->getPtr<uint8_t>()[sizeof(*header) + 0];
        DPRINTFS(DtuSysCalls, (&dtu), "  syscall: %s\n",
            sysNo < total ? syscallNames[sysNo] : "Unknown");
    }

    if (ep.msgCount < ep.size)
    {
        Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

        // Note that replyEpId is the Id of *our* sending EP
        if (header->flags & Dtu::REPLY_FLAG &&
            header->flags & Dtu::GRANT_CREDITS_FLAG &&
            header->replyEpId < dtu.numEndpoints)
        {
            SendEp sep = dtu.regs().getSendEp(header->replyEpId);

            sep.credits += sep.maxMsgSize;

            DPRINTFS(DtuCredits, (&dtu),
                "EP%u: received %u credits (%u in total)\n",
                header->replyEpId, sep.maxMsgSize, sep.credits);

            dtu.regs().setSendEp(header->replyEpId, sep);
        }

        // the message is transferred piece by piece; we can start as soon as
        // we have the header
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;
        delay += dtu.nocToTransferLatency;

        dtu.startTransfer(Dtu::TransferType::REMOTE_WRITE,
                          NocAddr(0, 0),
                          localAddr,
                          pkt->getSize(),
                          pkt,
                          NULL,
                          delay);

        incrementWritePtr(epId);
    }
    // ignore messages if there is not enough space
    else
    {
        DPRINTFS(Dtu, (&dtu), "EP%u: ignoring message: no space left\n", epId);

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
}
