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
#include "debug/DtuMsgs.hh"
#include "mem/dtu/msg_unit.hh"
#include "mem/dtu/noc_addr.hh"
#include "mem/dtu/xfer_unit.hh"

static const char *syscallNames[] = {
    "PAGEFAULT",
    "CREATE_SRV",
    "CREATE_SESS",
    "CREATE_RGATE",
    "CREATE_SGATE",
    "CREATE_MGATE",
    "CREATE_MAP",
    "CREATE_VPEGRP",
    "CREATE_VPE",
    "ACTIVATE",
    "SRV_CTRL",
    "VPE_CTRL",
    "VPE_WAIT",
    "DERIVE_MEM",
    "OPEN_SESS",
    "DELEGATE",
    "OBTAIN",
    "EXCHANGE",
    "REVOKE",
    "FORWARD_MSG",
    "FORWARD_MEM",
    "FORWARD_REPLY",
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
MessageUnit::startTransmission(const Dtu::Command::Bits& cmd)
{
    unsigned epid = cmd.epid;

    // if we want to reply, request the header first
    if (cmd.opcode == Dtu::Command::REPLY)
    {
        RecvEp ep = dtu.regs().getRecvEp(epid);
        int msgidx = ep.msgToIdx(cmd.arg);
        auto hd = dtu.regs().getHeader(ep.header + msgidx, RegAccess::DTU);

        if (!(hd.flags & Dtu::REPLY_ENABLED))
        {
            DPRINTFS(Dtu, (&dtu),
                "EP%u: double reply for msg %p?\n", epid, cmd.arg);
            dtu.scheduleFinishOp(Cycles(1), Dtu::Error::REPLY_DISABLED);
        }

        // now that we have the header, fill the info struct
        hd.flags &= ~Dtu::REPLY_ENABLED;
        dtu.regs().setHeader(ep.header + msgidx, RegAccess::DTU, hd);

        info.targetCoreId = hd.senderCoreId;
        info.targetEpId   = hd.replyEpId;  // send message to the reply EP
        info.replyEpId    = hd.senderEpId; // and grant credits to the sender

        // the receiver of the reply should get the label that he has set
        info.label        = hd.replyLabel;
        // replies don't have replies. so, we don't need that
        info.replyLabel   = 0;
        // the pagefault flag is moved to the reply hd
        info.flags        = hd.flags & Dtu::PAGEFAULT;
        info.unlimcred    = false;
        info.ready        = true;
    }
    else
    {
        // check if we have enough credits
        const DataReg data = dtu.regs().getDataReg();
        SendEp ep = dtu.regs().getSendEp(epid);

        if (ep.maxMsgSize == 0)
        {
            DPRINTFS(Dtu, (&dtu), "EP%u: invalid EP\n", epid);
            dtu.scheduleFinishOp(Cycles(1), Dtu::Error::INV_EP);
            return;
        }

        if (ep.curcrd != Dtu::CREDITS_UNLIM)
        {
            if (ep.curcrd < ep.maxMsgSize)
            {
                DPRINTFS(Dtu, (&dtu),
                    "EP%u: not enough credits (%lu) to send message (%lu)\n",
                    epid, ep.curcrd, ep.maxMsgSize);
                dtu.scheduleFinishOp(Cycles(1), Dtu::Error::MISS_CREDITS);
                return;
            }

            // pay the credits
            ep.curcrd -= ep.maxMsgSize;

            DPRINTFS(DtuCredits, (&dtu), "EP%u paid %u credits (%u left)\n",
                     epid, ep.maxMsgSize, ep.curcrd);

            dtu.regs().setSendEp(epid, ep);
        }

        // TODO error handling
        assert(data.size + sizeof(MessageHeader) <= ep.maxMsgSize);

        // fill the info struct and start the transfer
        info.targetCoreId = ep.targetCore;
        info.targetEpId   = ep.targetEp;
        info.label        = ep.label;
        info.replyLabel   = dtu.regs().get(CmdReg::REPLY_LABEL);
        info.replyEpId    = cmd.arg;
        info.flags        = 0;
        info.unlimcred    = ep.curcrd == Dtu::CREDITS_UNLIM;
        info.ready        = true;
    }

    startXfer(cmd);
}

void
MessageUnit::startXfer(const Dtu::Command::Bits& cmd)
{
    assert(info.ready);

    const DataReg data = dtu.regs().getDataReg();

    if (cmd.opcode == Dtu::Command::REPLY)
        repliedBytes.sample(data.size);
    else
        sentBytes.sample(data.size);

    DPRINTFS(Dtu, (&dtu), "\e[1m[%s -> %u]\e[0m with EP%u of %#018lx:%lu\n",
             cmd.opcode == Dtu::Command::REPLY ? "rp" : "sd",
             info.targetCoreId,
             cmd.epid,
             data.addr,
             data.size);

    MessageHeader* header = new MessageHeader;

    if (cmd.opcode == Dtu::Command::REPLY)
        header->flags = Dtu::REPLY_FLAG | Dtu::GRANT_CREDITS_FLAG;
    else
        header->flags = Dtu::REPLY_ENABLED; // normal message
    header->flags |= info.flags;

    header->senderCoreId = dtu.coreId;
    header->senderEpId   = info.unlimcred ? dtu.numEndpoints : cmd.epid;
    header->replyEpId    = info.replyEpId;
    header->length       = data.size;
    header->label        = info.label;
    header->replyLabel   = info.replyLabel;

    DPRINTFS(Dtu, (&dtu),
        "  src: pe=%u ep=%u rpep=%u rplbl=%#018lx flags=%#x%s\n",
        header->senderCoreId, header->senderEpId,
        header->replyEpId, info.replyLabel, header->flags,
        header->senderCoreId != dtu.coreId ? " (on behalf)" : "");

    DPRINTFS(Dtu, (&dtu),
        "  dst: pe=%u ep=%u lbl=%#018lx\n",
        info.targetCoreId, info.targetEpId, info.label);

    assert(data.size + sizeof(MessageHeader) <= dtu.maxNocPacketSize);

    NocAddr nocAddr(info.targetCoreId, info.targetEpId);
    uint flags = XferUnit::MESSAGE;

    // start the transfer of the payload
    auto *ev = new SendTransferEvent(
        data.addr, data.size, flags, nocAddr, header);
    dtu.startTransfer(ev, dtu.startMsgTransferDelay);

    info.ready = false;
}

void
MessageUnit::SendTransferEvent::transferStart()
{
    assert(header);

    // note that this causes no additional delay because we assume that we
    // create the header directly in the buffer (and if there is no one
    // free we just wait until there is)
    memcpy(data(), header, sizeof(*header));

    // for the header
    size(sizeof(*header));

    delete header;
    header = nullptr;
}

void
MessageUnit::finishMsgReply(Dtu::Error error, unsigned epid, Addr msgAddr)
{
    if (error == Dtu::Error::VPE_GONE)
    {
        RecvEp ep = dtu.regs().getRecvEp(epid);
        int msgidx = ep.msgToIdx(msgAddr);

        ReplyHeader hd = dtu.regs().getHeader(ep.header + msgidx, RegAccess::DTU);
        hd.flags |= Dtu::REPLY_FAILED;
        dtu.regs().setHeader(ep.header + msgidx, RegAccess::DTU, hd);
    }
    // on VPE_GONE, the kernel wants to reply later; so don't free the slot
    else
        ackMessage(epid, msgAddr);
}

void
MessageUnit::finishMsgSend(Dtu::Error error, unsigned epid)
{
    SendEp ep = dtu.regs().getSendEp(epid);
    // don't do anything if the EP is invalid
    if (ep.maxMsgSize == 0)
        return;

    // undo the credit reduction on errors except for {VPE_GONE,MISS_CREDITS}
    if (ep.curcrd != Dtu::CREDITS_UNLIM &&
        error != Dtu::Error::NONE && error != Dtu::Error::VPE_GONE &&
        error != Dtu::Error::MISS_CREDITS)
    {
        ep.curcrd += ep.maxMsgSize;
        assert(ep.curcrd <= ep.maxcrd);
    }

    dtu.regs().setSendEp(epid, ep);
}

void
MessageUnit::recvCredits(unsigned epid)
{
    SendEp ep = dtu.regs().getSendEp(epid);

    if (ep.curcrd != Dtu::CREDITS_UNLIM)
    {
        ep.curcrd += ep.maxMsgSize;
        assert(ep.curcrd <= ep.maxcrd);

        DPRINTFS(DtuCredits, (&dtu),
            "EP%u received %u credits (%u in total)\n",
            epid, ep.maxMsgSize, ep.curcrd);

        dtu.regs().setSendEp(epid, ep);
    }
}

Addr
MessageUnit::fetchMessage(unsigned epid)
{
    RecvEp ep = dtu.regs().getRecvEp(epid);

    if (ep.msgCount == 0)
        return 0;

    int i;
    for (i = ep.rdPos; i < (1 << ep.size); ++i)
    {
        if (ep.isUnread(i))
            goto found;
    }
    for (i = 0; i < ep.rdPos; ++i)
    {
        if (ep.isUnread(i))
            goto found;
    }

    // should not get here
    assert(false);

found:
    assert(ep.isOccupied(i));

    ep.setUnread(i, false);
    ep.msgCount--;
    ep.rdPos = i + 1;

    DPRINTFS(DtuBuf, (&dtu),
        "EP%u: fetched message at index %u (count=%u)\n",
        epid, i, ep.msgCount);

    dtu.regs().setRecvEp(epid, ep);

    return ep.bufAddr + (i << ep.msgSize);
}

int
MessageUnit::allocSlot(size_t msgSize, unsigned epid, RecvEp &ep)
{
    // the RecvEp might be invalid
    if (ep.bufAddr == 0)
        return -1;

    assert(msgSize <= (1 << ep.msgSize));

    int i;
    for (i = ep.wrPos; i < (1 << ep.size); ++i)
    {
        if (!ep.isOccupied(i))
            goto found;
    }
    for (i = 0; i < ep.wrPos; ++i)
    {
        if (!ep.isOccupied(i))
            goto found;
    }

    return -1;

found:
    ep.setOccupied(i, true);
    ep.wrPos = i + 1;

    DPRINTFS(DtuBuf, (&dtu),
        "EP%u: put message at index %u\n",
        epid, i);

    dtu.regs().setRecvEp(epid, ep);
    return i;
}

Dtu::Error
MessageUnit::ackMessage(unsigned epId, Addr msgAddr)
{
    RecvEp ep = dtu.regs().getRecvEp(epId);

    int msgidx = ep.msgToIdx(msgAddr);
    if (msgidx == RecvEp::MAX_MSGS || !ep.isOccupied(msgidx))
        return Dtu::Error::INV_MSG;

    ep.setOccupied(msgidx, false);
    if (ep.isUnread(msgidx))
    {
        ep.setUnread(msgidx, false);
        ep.msgCount--;
    }

    // reset header
    dtu.regs().setHeader(ep.header + msgidx, RegAccess::DTU, ReplyHeader());

    DPRINTFS(DtuBuf, (&dtu),
        "EP%u: acked msg at index %d\n",
        epId, msgidx);

    dtu.regs().setRecvEp(epId, ep);
    return Dtu::Error::NONE;
}

Dtu::Error
MessageUnit::invalidateReply(unsigned repId, unsigned peId, unsigned sepId)
{
    RecvEp ep = dtu.regs().getRecvEp(repId);
    if (ep.bufAddr == 0)
        return Dtu::Error::INV_EP;

    for (int i = 0; i < (1 << ep.size); ++i)
    {
        auto hd = dtu.regs().getHeader(ep.header + i, RegAccess::DTU);
        if (hd.senderCoreId == peId && hd.senderEpId == sepId)
            dtu.regs().setHeader(ep.header + i, RegAccess::DTU, ReplyHeader());
    }
    return Dtu::Error::NONE;
}

Dtu::Error
MessageUnit::finishMsgReceive(unsigned epId,
                              Addr msgAddr,
                              const MessageHeader *header,
                              Dtu::Error error,
                              uint xferFlags)
{
    RecvEp ep = dtu.regs().getRecvEp(epId);
    int idx = (msgAddr - ep.bufAddr) >> ep.msgSize;

    if (error == Dtu::Error::NONE)
    {
        // Note that replyEpId is the Id of *our* sending EP
        if (header->flags & Dtu::REPLY_FLAG &&
            header->flags & Dtu::GRANT_CREDITS_FLAG &&
            header->replyEpId < dtu.numEndpoints)
        {
            recvCredits(header->replyEpId);
        }

        DPRINTFS(DtuBuf, (&dtu),
            "EP%u: increment message count to %u\n",
            epId, ep.msgCount + 1);

        if (ep.msgCount == (1 << ep.size))
        {
            warn("EP%u: Buffer full!\n", epId);
            return error;
        }

        ep.msgCount++;
        ep.setUnread(idx, true);
        dtu.regs().setHeader(ep.header + idx, RegAccess::DTU, *header);
    }
    else
        ep.setOccupied(idx, false);

    dtu.regs().setRecvEp(epId, ep);

    if (error == Dtu::Error::NONE)
    {
        dtu.regs().setEvent(EventType::MSG_RECV);
        if (dtu.regs().hasFeature(Features::IRQ_ON_MSG))
            dtu.setIrq();
        else
            dtu.wakeupCore();
    }

    return error;
}

Dtu::Error
MessageUnit::recvFromNoc(PacketPtr pkt, uint flags)
{
    assert(pkt->isWrite());
    assert(pkt->hasData());

    MessageHeader* header = pkt->getPtr<MessageHeader>();

    receivedBytes.sample(header->length);

    uint8_t pfResp = Dtu::REPLY_FLAG | Dtu::PAGEFAULT;
    if ((header->flags & pfResp) == pfResp)
    {
        dtu.handlePFResp(pkt);
        return Dtu::Error::NONE;
    }

    NocAddr addr(pkt->getAddr());
    unsigned epId = addr.offset;

    DPRINTFS(Dtu, (&dtu),
        "\e[1m[rv <- %u]\e[0m %lu bytes on EP%u\n",
        header->senderCoreId, header->length, epId);
    dtu.printPacket(pkt);

    if (dtu.coreId == 0 && epId == 0 && DTRACE(DtuSysCalls))
    {
        const size_t total = (sizeof(syscallNames) / sizeof(syscallNames[0]));
        size_t sysNo = pkt->getPtr<uint8_t>()[sizeof(*header) + 0];
        DPRINTFS(DtuSysCalls, (&dtu), "  syscall: %s\n",
            sysNo < total ? syscallNames[sysNo] : "Unknown");
    }

    if (DTRACE(DtuMsgs))
    {
        uint64_t *words = reinterpret_cast<uint64_t*>(header + 1);
        for(size_t i = 0; i < header->length / sizeof(uint64_t); ++i)
            DPRINTFS(DtuMsgs, (&dtu), "    word%2lu: %#018x\n", i, words[i]);
    }

    // support credit receives without storing reply messages
    if (epId >= dtu.numEndpoints &&
        (header->flags & Dtu::REPLY_FLAG) &&
        (header->flags & Dtu::GRANT_CREDITS_FLAG) &&
        header->replyEpId < dtu.numEndpoints)
    {
        recvCredits(header->replyEpId);
        dtu.sendNocResponse(pkt);
        dtu.regs().setEvent(EventType::CRD_RECV);
        dtu.wakeupCore();
        return Dtu::Error::NONE;
    }

    RecvEp ep = dtu.regs().getRecvEp(epId);

    int msgidx = allocSlot(pkt->getSize(), epId, ep);
    if (msgidx == -1)
    {
        DPRINTFS(Dtu, (&dtu),
            "EP%u: ignoring message: no space left\n",
            epId);
        noSpace++;

        dtu.sendNocResponse(pkt);
        return Dtu::Error::NO_RING_SPACE;
    }

    // the message is transferred piece by piece; we can start as soon as
    // we have the header
    Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
    pkt->headerDelay = 0;
    delay += dtu.nocToTransferLatency;

    // atm, message receives can never cause pagefaults
    uint rflags = XferUnit::XferFlags::MSGRECV | XferUnit::XferFlags::NOPF;
    if (flags & Dtu::NocFlags::PRIV)
        rflags |= XferUnit::XferFlags::PRIV;
    Addr localAddr = ep.bufAddr + (msgidx << ep.msgSize);

    auto *ev = new ReceiveTransferEvent(this, localAddr, rflags, pkt);
    dtu.startTransfer(ev, delay);

    return Dtu::Error::NONE;
}

void
MessageUnit::ReceiveTransferEvent::transferDone(Dtu::Error result)
{
    MessageHeader* header = pkt->getPtr<MessageHeader>();
    NocAddr addr(pkt->getAddr());

    result = msgUnit->finishMsgReceive(addr.offset, msgAddr, header, result, flags());

    MemoryUnit::ReceiveTransferEvent::transferDone(result);
}
