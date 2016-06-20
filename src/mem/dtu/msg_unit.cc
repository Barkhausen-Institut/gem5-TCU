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
#include "mem/dtu/xfer_unit.hh"

static const char *syscallNames[] = {
    "PAGEFAULT",
    "CREATESRV",
    "CREATESESS",
    "CREATESESSAT",
    "CREATEGATE",
    "CREATEVPE",
    "CREATEMAP",
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
MessageUnit::regStats()
{
    sentBytes
        .init(8)
        .name(dtu.name() + ".msg.sentBytes")
        .desc("Sent messages (in bytes)")
        .flags(Stats::nozero);
    repliedBytes
        .init(8)
        .name(dtu.name() + ".msg.repliedBytes")
        .desc("Sent replies (in bytes)")
        .flags(Stats::nozero);
    receivedBytes
        .init(8)
        .name(dtu.name() + ".msg.receivedBytes")
        .desc("Received messages (in bytes)")
        .flags(Stats::nozero);
    wrongVPE
        .name(dtu.name() + ".msg.wrongVPE")
        .desc("Number of received messages that targeted the wrong VPE")
        .flags(Stats::nozero);
    noSpace
        .name(dtu.name() + ".msg.noSpace")
        .desc("Number of received messages we dropped")
        .flags(Stats::nozero);
}

void
MessageUnit::startTransmission(const Dtu::Command& cmd)
{
    unsigned epid = cmd.arg;

    // if we want to reply, request the header first
    if (cmd.opcode == Dtu::Command::REPLY)
    {
        offset = 0;
        flagsPhys = 0;
        requestHeader(cmd.arg);
        return;
    }

    // check if we have enough credits
    Addr messageSize = dtu.regs().get(CmdReg::DATA_SIZE);
    SendEp ep = dtu.regs().getSendEp(epid);

    // TODO error handling
    assert(messageSize + sizeof(Dtu::MessageHeader) <= ep.maxMsgSize);

    if (ep.credits != Dtu::CREDITS_UNLIM)
    {
        if (ep.credits < ep.maxMsgSize)
        {
            DPRINTFS(Dtu, (&dtu),
                "EP%u: not enough credits (%lu) to send message (%lu)\n",
                epid, ep.credits, ep.maxMsgSize);
            dtu.scheduleFinishOp(Cycles(1), Dtu::Error::MISS_CREDITS);
            return;
        }

        ep.credits -= ep.maxMsgSize;

        DPRINTFS(DtuCredits, (&dtu), "EP%u pays %u credits (%u left)\n",
                 epid, ep.maxMsgSize, ep.credits);

        // pay the credits
        dtu.regs().setSendEp(epid, ep);
    }

    // fill the info struct and start the transfer
    info.targetCoreId = ep.targetCore;
    info.targetVpeId  = ep.vpeId;
    info.targetEpId   = ep.targetEp;
    info.label        = ep.label;
    info.replyLabel   = dtu.regs().get(CmdReg::REPLY_LABEL);
    info.replyEpId    = dtu.regs().get(CmdReg::REPLY_EPID);
    info.flags        = 0;
    info.unlimcred    = ep.credits == Dtu::CREDITS_UNLIM;
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
            DPRINTFS(DtuTlb, (&dtu),
                "%s for read access to %p\n",
                res != DtuTlb::MISS ? "Pagefault" : "TLB-miss",
                msgAddr);
            assert(res != DtuTlb::NOMAP);

            Translation *trans = new Translation(*this, msgAddr, epid);
            dtu.startTranslate(msgAddr, DtuTlb::READ, trans);
            return;
        }
    }

    requestHeaderWithPhys(epid, true, msgAddr, phys);
}

void
MessageUnit::requestHeaderWithPhys(unsigned epid,
                                   bool success,
                                   Addr virt,
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
                       virt,
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
        requestHeader(cmd.arg);
        return;
    }

    // now that we have the header, fill the info struct
    assert(header.flags & Dtu::REPLY_ENABLED);

    info.targetCoreId = header.senderCoreId;
    info.targetVpeId  = header.senderVpeId;
    info.targetEpId   = header.replyEpId;  // send message to the reply EP
    info.replyEpId    = header.senderEpId; // and grant credits to the sender

    // the receiver of the reply should get the label that he has set
    info.label        = header.replyLabel;
    // replies don't have replies. so, we don't need that
    info.replyLabel   = 0;
    // the pagefault flag is moved to the reply header
    info.flags        = header.flags & Dtu::PAGEFAULT;
    info.unlimcred    = false;
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

    if (cmd.opcode == Dtu::Command::REPLY)
        repliedBytes.sample(messageSize);
    else
        sentBytes.sample(messageSize);

    DPRINTFS(Dtu, (&dtu), "\e[1m[%s -> %u]\e[0m with EP%u of %#018lx:%lu\n",
             cmd.opcode == Dtu::Command::REPLY ? "rp" : "sd",
             info.targetCoreId,
             cmd.arg,
             dtu.regs().get(CmdReg::DATA_ADDR),
             messageSize);

    DPRINTFS(Dtu, (&dtu),
        "  header: flags=%#x tgtVPE=%u tgtEP=%u lbl=%#018lx"
        " rpLbl=%#018lx rpEP=%u\n",
        info.flags, info.targetVpeId, info.targetEpId, info.label,
        info.replyLabel, info.replyEpId);

    Dtu::MessageHeader* header = new Dtu::MessageHeader;

    if (cmd.opcode == Dtu::Command::REPLY)
        header->flags = Dtu::REPLY_FLAG | Dtu::GRANT_CREDITS_FLAG;
    else
        header->flags = Dtu::REPLY_ENABLED; // normal message
    header->flags |= info.flags;

    header->senderCoreId = dtu.coreId;
    header->senderVpeId  = dtu.regs().get(DtuReg::VPE_ID);
    header->senderEpId   = info.unlimcred ? dtu.numEndpoints : cmd.arg;
    header->replyEpId    = info.replyEpId;
    header->length       = messageSize;
    header->label        = info.label;
    header->replyLabel   = info.replyLabel;

    assert(messageSize + sizeof(Dtu::MessageHeader) <= dtu.maxNocPacketSize);

    // start the transfer of the payload
    NocAddr nocAddr(info.targetCoreId, info.targetEpId);
    dtu.startTransfer(Dtu::TransferType::LOCAL_READ,
                      nocAddr,
                      messageAddr,
                      messageSize,
                      NULL,
                      info.targetVpeId,
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
        "EP%u: increment write pointer to %#018lx\n",
        epId, ep.wrOff);

    dtu.regs().setRecvEp(epId, ep);
    return true;
}

void
MessageUnit::incrementMsgCnt(unsigned epId)
{
    RecvEp ep = dtu.regs().getRecvEp(epId);

    DPRINTFS(DtuBuf, (&dtu),
        "EP%u: increment message count to %u\n",
        epId, ep.msgCount + 1);

    if (ep.msgCount == ep.size)
    {
        warn("EP%u: Buffer full!\n", epId);
        return;
    }
    ep.msgCount++;

    dtu.regs().setRecvEp(epId, ep);

    dtu.updateSuspendablePin();
    dtu.wakeupCore();
}

Dtu::Error
MessageUnit::recvFromNoc(PacketPtr pkt, uint vpeId)
{
    assert(pkt->isWrite());
    assert(pkt->hasData());

    Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

    receivedBytes.sample(header->length);

    uint8_t pfResp = Dtu::REPLY_FLAG | Dtu::PAGEFAULT;
    if ((header->flags & pfResp) == pfResp)
    {
        dtu.handlePFResp(pkt);
        return Dtu::Error::NONE;
    }

    NocAddr addr(pkt->getAddr());
    unsigned epId = addr.offset;
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

    Dtu::Error res = Dtu::Error::NONE;
    uint16_t ourVpeId = dtu.regs().get(DtuReg::VPE_ID);
    if (vpeId == ourVpeId && ep.msgCount < ep.size)
    {
        Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

        // Note that replyEpId is the Id of *our* sending EP
        if (header->flags & Dtu::REPLY_FLAG &&
            header->flags & Dtu::GRANT_CREDITS_FLAG &&
            header->replyEpId < dtu.numEndpoints)
        {
            SendEp sep = dtu.regs().getSendEp(header->replyEpId);

            if (sep.credits != Dtu::CREDITS_UNLIM)
            {
                sep.credits += sep.maxMsgSize;

                DPRINTFS(DtuCredits, (&dtu),
                    "EP%u: received %u credits (%u in total)\n",
                    header->replyEpId, sep.maxMsgSize, sep.credits);

                dtu.regs().setSendEp(header->replyEpId, sep);
            }
        }

        // the message is transferred piece by piece; we can start as soon as
        // we have the header
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;
        delay += dtu.nocToTransferLatency;

        // atm, message receives can never cause pagefaults
        uint flags = XferUnit::XferFlags::MSGRECV | XferUnit::XferFlags::NOPF;
        dtu.startTransfer(Dtu::TransferType::REMOTE_WRITE,
                          NocAddr(0, 0),
                          localAddr,
                          pkt->getSize(),
                          pkt,
                          0,
                          NULL,
                          delay,
                          flags);

        incrementWritePtr(epId);
    }
    // ignore messages for other VPEs or if there is not enough space
    else
    {
        if (vpeId != ourVpeId)
        {
            DPRINTFS(Dtu, (&dtu),
                "EP%u: received message for VPE %u, but VPE %u is running\n",
                epId, vpeId, ourVpeId);
            res = Dtu::Error::VPE_GONE;
            wrongVPE++;
        }
        else
        {
            DPRINTFS(Dtu, (&dtu),
                "EP%u: ignoring message: no space left\n",
                epId);
            res = Dtu::Error::NO_RING_SPACE;
            noSpace++;
        }

        dtu.sendNocResponse(pkt);
    }

    return res;
}
