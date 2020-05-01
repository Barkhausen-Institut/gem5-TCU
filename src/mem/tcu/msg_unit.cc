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

#include "debug/Tcu.hh"
#include "debug/TcuBuf.hh"
#include "debug/TcuCredits.hh"
#include "debug/TcuPackets.hh"
#include "debug/TcuMsgs.hh"
#include "mem/tcu/msg_unit.hh"
#include "mem/tcu/noc_addr.hh"
#include "mem/tcu/xfer_unit.hh"

void
MessageUnit::regStats()
{
    sentBytes
        .init(8)
        .name(tcu.name() + ".msg.sentBytes")
        .desc("Sent messages (in bytes)")
        .flags(Stats::nozero);
    repliedBytes
        .init(8)
        .name(tcu.name() + ".msg.repliedBytes")
        .desc("Sent replies (in bytes)")
        .flags(Stats::nozero);
    receivedBytes
        .init(8)
        .name(tcu.name() + ".msg.receivedBytes")
        .desc("Received messages (in bytes)")
        .flags(Stats::nozero);
    wrongVPE
        .name(tcu.name() + ".msg.wrongVPE")
        .desc("Number of received messages that targeted the wrong VPE")
        .flags(Stats::nozero);
    noSpace
        .name(tcu.name() + ".msg.noSpace")
        .desc("Number of received messages we dropped")
        .flags(Stats::nozero);
}

void
MessageUnit::startTransmission(const CmdCommand::Bits& cmd)
{
    epid_t epid = cmd.epid;

    info.sepId = Tcu::INVALID_EP_ID;

    // if we want to reply, load the reply EP first
    if (cmd.opcode == CmdCommand::REPLY)
    {
        RecvEp *ep = tcu.regs().getRecvEp(epid);

        if(!ep || ep->r0.vpe != tcu.regs().getVPE())
        {
            DPRINTFS(Tcu, (&tcu), "EP%u: invalid EP\n", epid);
            tcu.scheduleFinishOp(Cycles(1), TcuError::INV_EP);
            return;
        }

        if (ep->r0.replyEps == Tcu::INVALID_EP_ID)
        {
            DPRINTFS(Tcu, (&tcu),
                     "EP%u: no reply EPs, cannot reply on msg %p\n",
                     epid, cmd.arg);
            tcu.scheduleFinishOp(Cycles(1), TcuError::INV_EP);
            return;
        }

        int msgidx = ep->offsetToIdx(cmd.arg);
        epid = ep->r0.replyEps + msgidx;

        SendEp *sep = tcu.regs().getSendEp(epid);

        if (!sep || !(sep->r0.flags & SendEp::FL_REPLY))
        {
            DPRINTFS(Tcu, (&tcu),
                     "EP%u: invalid reply EP. Double reply for msg %p?\n",
                     epid, cmd.arg);
            tcu.scheduleFinishOp(Cycles(1), TcuError::INV_ARGS);
            return;
        }

        assert(sep->r0.vpe == tcu.regs().getVPE());

        // grant credits to the sender
        info.replyEpId = sep->r0.crdEp;
        info.flags = Tcu::REPLY_FLAG;
        info.replySize = 0;
    }
    else
    {
        info.flags = 0;
        info.replyEpId = Tcu::INVALID_EP_ID;
        info.replySize = ceil(log2(sizeof(MessageHeader)));
    }

    const CmdData::Bits data = tcu.regs().getData();
    SendEp *ep = tcu.regs().getSendEp(epid);

    // get info from reply EP
    if (cmd.opcode == CmdCommand::SEND && cmd.arg != Tcu::INVALID_EP_ID)
    {
        RecvEp *rep = tcu.regs().getRecvEp(cmd.arg);
        if (!rep || rep->r0.vpe != tcu.regs().getVPE())
        {
            DPRINTFS(Tcu, (&tcu), "EP%u: invalid reply EP\n", cmd.arg);
            tcu.scheduleFinishOp(Cycles(1), TcuError::INV_EP);
            return;
        }

        info.replyEpId = cmd.arg;
        info.replySize = rep->r0.msgSize;
    }

    // check if the send EP is valid
    if (!ep || ep->r0.vpe != tcu.regs().getVPE() ||
        (cmd.opcode == CmdCommand::SEND && ep->r0.flags != 0))
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: invalid EP\n", epid);
        tcu.scheduleFinishOp(Cycles(1), TcuError::INV_EP);
        return;
    }

    // check message size
    if (data.size + sizeof(MessageHeader) > (1 << ep->r0.maxMsgSize))
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: message too large\n", epid);
        tcu.scheduleFinishOp(Cycles(1), TcuError::INV_ARGS);
        return;
    }

    // check if we have enough credits
    if (ep->r0.curcrd != Tcu::CREDITS_UNLIM)
    {
        if (ep->r0.curcrd == 0)
        {
            DPRINTFS(Tcu, (&tcu), "EP%u: no credits to send message\n", epid);
            tcu.scheduleFinishOp(Cycles(1), TcuError::MISS_CREDITS);
            return;
        }

        // pay the credits
        ep->r0.curcrd = ep->r0.curcrd - 1;

        DPRINTFS(TcuCredits, (&tcu), "EP%u paid 1 credit (%u left)\n",
                 epid, ep->r0.curcrd);

        tcu.regs().updateEp(epid);
    }

    // fill the info struct and start the transfer
    info.targetPeId   = ep->r1.targetPe;
    info.targetEpId   = ep->r1.targetEp;
    info.label        = ep->r2.label;
    info.replyLabel   = tcu.regs().get(CmdReg::ARG1);
    info.unlimcred    = ep->r0.curcrd == Tcu::CREDITS_UNLIM;
    info.ready        = true;
    info.sepId        = epid;

    startXfer(cmd);
}

void
MessageUnit::startXfer(const CmdCommand::Bits& cmd)
{
    assert(info.ready);

    const CmdData::Bits data = tcu.regs().getData();

    if (cmd.opcode == CmdCommand::REPLY)
        repliedBytes.sample(data.size);
    else
        sentBytes.sample(data.size);

    DPRINTFS(Tcu, (&tcu), "\e[1m[%s -> %u]\e[0m with EP%u of %#018lx:%lu\n",
             cmd.opcode == CmdCommand::REPLY ? "rp" : "sd",
             info.targetPeId,
             cmd.epid,
             data.addr,
             data.size);

    MessageHeader* header = new MessageHeader;

    if (cmd.opcode == CmdCommand::REPLY)
        header->flags = Tcu::REPLY_FLAG;
    else
        header->flags = 0; // normal message
    header->flags |= info.flags;

    header->senderPeId   = tcu.peId;
    header->senderEpId   = info.unlimcred ? Tcu::INVALID_EP_ID : cmd.epid;
    header->replyEpId    = info.replyEpId;
    header->length       = data.size;
    header->label        = info.label;
    header->replyLabel   = info.replyLabel;
    header->replySize    = info.replySize;

    DPRINTFS(Tcu, (&tcu),
        "  src: pe=%u ep=%u rpep=%u rplbl=%#018lx rpsize=%#x flags=%#x%s\n",
        header->senderPeId, header->senderEpId, header->replyEpId,
        header->replyLabel, 1 << header->replySize, header->flags,
        header->senderPeId != tcu.peId ? " (on behalf)" : "");

    DPRINTFS(Tcu, (&tcu),
        "  dst: pe=%u ep=%u lbl=%#018lx\n",
        info.targetPeId, info.targetEpId, info.label);

    assert(data.size + sizeof(MessageHeader) <= tcu.maxNocPacketSize);

    NocAddr nocAddr(info.targetPeId, info.targetEpId);
    uint flags = XferUnit::MESSAGE;

    // start the transfer of the payload
    auto *ev = new SendTransferEvent(
        data.addr, data.size, flags, nocAddr, header);
    tcu.startTransfer(ev, tcu.startMsgTransferDelay);

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
MessageUnit::finishMsgReply(TcuError error, epid_t epid, Addr msgOff)
{
    if (error == TcuError::NONE)
        ackMessage(epid, msgOff);
    // undo credit reduction
    else if (info.sepId != Tcu::INVALID_EP_ID)
        recvCredits(info.sepId);
}

void
MessageUnit::finishMsgSend(TcuError error, epid_t epid)
{
    if (error != TcuError::NONE && info.sepId != Tcu::INVALID_EP_ID)
        recvCredits(info.sepId);
}

void
MessageUnit::recvCredits(epid_t epid)
{
    SendEp *ep = tcu.regs().getSendEp(epid);
    // don't do anything if the EP is invalid
    if (!ep)
        return;

    if (ep->r0.curcrd != Tcu::CREDITS_UNLIM)
    {
        ep->r0.curcrd = ep->r0.curcrd + 1;
        assert(ep->r0.curcrd <= ep->r0.maxcrd);

        DPRINTFS(TcuCredits, (&tcu),
            "EP%u received 1 credit (%u in total)\n",
            epid, ep->r0.curcrd);

        tcu.regs().updateEp(epid);
    }
}

TcuError
MessageUnit::fetchMessage(epid_t epid, Addr *msgOff)
{
    RecvEp *ep = tcu.regs().getRecvEp(epid);
    if (!ep || ep->r0.vpe != tcu.regs().getVPE())
        return TcuError::INV_EP;

    // check if the current VPE has unread messages at all. note that this is
    // important in case it is out of sync with the receive EPs, i.e., if we
    // have ongoing foreignRecv core requests.
    if (ep->r2.unread == 0 || tcu.regs().messages() == 0)
    {
        *msgOff = -1;
        return TcuError::NONE;
    }

    int i;
    for (i = ep->r0.rdPos; i < (1 << ep->r0.size); ++i)
    {
        if (ep->isUnread(i))
            goto found;
    }
    for (i = 0; i < ep->r0.rdPos; ++i)
    {
        if (ep->isUnread(i))
            goto found;
    }

    // should not get here
    assert(false);

found:
    assert(ep->isOccupied(i));

    ep->setUnread(i, false);
    ep->r0.rdPos = i + 1;

    DPRINTFS(TcuBuf, (&tcu),
        "EP%u: fetched message at index %u (count=%u)\n",
        epid, i, ep->unreadMsgs());

    tcu.regs().updateEp(epid);
    tcu.regs().rem_msg();

    *msgOff = i << ep->r0.msgSize;
    return TcuError::NONE;
}

int
MessageUnit::allocSlot(size_t msgSize, epid_t epid, RecvEp *ep)
{
    // the RecvEp might be invalid
    if (!ep)
        return -1;

    assert(msgSize <= (1 << ep->r0.msgSize));

    int i;
    for (i = ep->r0.wrPos; i < (1 << ep->r0.size); ++i)
    {
        if (!ep->isOccupied(i))
            goto found;
    }
    for (i = 0; i < ep->r0.wrPos; ++i)
    {
        if (!ep->isOccupied(i))
            goto found;
    }

    return -1;

found:
    ep->setOccupied(i, true);
    ep->r0.wrPos = i + 1;

    DPRINTFS(TcuBuf, (&tcu),
        "EP%u: put message at index %u\n",
        epid, i);

    tcu.regs().updateEp(epid);
    return i;
}

TcuError
MessageUnit::ackMessage(epid_t epId, Addr msgOff)
{
    RecvEp *ep = tcu.regs().getRecvEp(epId);
    if (!ep || ep->r0.vpe != tcu.regs().getVPE())
        return TcuError::INV_EP;

    int msgidx = ep->offsetToIdx(msgOff);
    if (msgidx == RecvEp::MAX_MSGS || !ep->isOccupied(msgidx))
        return TcuError::INV_MSG;

    bool unread = false;
    ep->setOccupied(msgidx, false);
    if (ep->isUnread(msgidx))
    {
        ep->setUnread(msgidx, false);
        unread = true;
    }

    if (ep->r0.replyEps != Tcu::INVALID_EP_ID)
    {
        // invalidate reply EP
        unsigned unread_mask;
        tcu.regs().invalidate(ep->r0.replyEps + msgidx, true, &unread_mask);
    }

    DPRINTFS(TcuBuf, (&tcu),
        "EP%u: acked msg at index %d\n",
        epId, msgidx);

    tcu.regs().updateEp(epId);
    if (unread)
        tcu.regs().rem_msg();
    return TcuError::NONE;
}

TcuError
MessageUnit::invalidateReply(epid_t repId, peid_t peId, epid_t sepId)
{
    RecvEp *ep = tcu.regs().getRecvEp(repId);
    if (!ep || ep->r0.replyEps == Tcu::INVALID_EP_ID)
        return TcuError::INV_EP;

    for (epid_t i = 0; i < (1 << ep->r0.size); ++i)
    {
        auto *sep = tcu.regs().getSendEp(ep->r0.replyEps + i);
        if (sep && sep->r1.targetPe == peId && sep->r0.crdEp == sepId)
        {
            unsigned unread_mask;
            tcu.regs().invalidate(ep->r0.replyEps + i, true, &unread_mask);
        }
    }
    return TcuError::NONE;
}

TcuError
MessageUnit::finishMsgReceive(epid_t epId,
                              Addr msgAddr,
                              const MessageHeader *header,
                              TcuError error,
                              uint xferFlags,
                              bool addMsg)
{
    RecvEp *ep = tcu.regs().getRecvEp(epId);
    if (!ep)
        return TcuError::INV_EP;

    int idx = (msgAddr - ep->r1.bufAddr) >> ep->r0.msgSize;

    if (error == TcuError::NONE)
    {
        // Note that replyEpId is the Id of *our* sending EP
        if (header->flags & Tcu::REPLY_FLAG &&
            header->replyEpId != Tcu::INVALID_EP_ID)
        {
            recvCredits(header->replyEpId);
        }

        DPRINTFS(TcuBuf, (&tcu),
            "EP%u: increment message count to %u\n",
            epId, ep->unreadMsgs() + 1);

        ep->setUnread(idx, true);

        if (!(header->flags & Tcu::REPLY_FLAG) &&
            ep->r0.replyEps != Tcu::INVALID_EP_ID &&
            header->replyEpId != Tcu::INVALID_EP_ID)
        {
            // install use-once reply EP
            Ep *sep = tcu.regs().getEp(ep->r0.replyEps + idx);
            sep->send.r0.type = static_cast<uint>(EpType::SEND);
            sep->send.r0.vpe = ep->r0.vpe;
            sep->send.r0.maxMsgSize = header->replySize;
            sep->send.r0.maxcrd = sep->send.r0.curcrd = 1;
            sep->send.r0.crdEp = header->senderEpId;
            sep->send.r0.flags = SendEp::FL_REPLY;
            sep->send.r1.targetPe = header->senderPeId;
            sep->send.r1.targetEp = header->replyEpId;
            sep->send.r2.label = header->replyLabel;
            tcu.regs().updateEp(ep->r0.replyEps + idx);
        }
    }
    else
        ep->setOccupied(idx, false);

    tcu.regs().updateEp(epId);

    if (error == TcuError::NONE && addMsg)
    {
        tcu.regs().add_msg();
        tcu.wakeupCore(false);
    }

    return error;
}

TcuError
MessageUnit::recvFromNoc(PacketPtr pkt)
{
    assert(pkt->isWrite());
    assert(pkt->hasData());

    MessageHeader* header = pkt->getPtr<MessageHeader>();

    receivedBytes.sample(header->length);

    NocAddr addr(pkt->getAddr());
    epid_t epId = addr.offset;
    assert(epId != Tcu::INVALID_EP_ID);

    DPRINTFS(Tcu, (&tcu),
        "\e[1m[rv <- %u]\e[0m %lu bytes on EP%u\n",
        header->senderPeId, header->length, epId);
    tcu.printPacket(pkt);

    if (DTRACE(TcuMsgs))
    {
        uint64_t *words = reinterpret_cast<uint64_t*>(header + 1);
        for(size_t i = 0; i < header->length / sizeof(uint64_t); ++i)
            DPRINTFS(TcuMsgs, (&tcu), "    word%2lu: %#018x\n", i, words[i]);
    }

    RecvEp *ep = tcu.regs().getRecvEp(epId);

    int msgidx = allocSlot(pkt->getSize(), epId, ep);
    if (msgidx == -1)
    {
        DPRINTFS(Tcu, (&tcu),
            "EP%u: ignoring message: no space left\n",
            epId);
        noSpace++;

        tcu.sendNocResponse(pkt);
        return TcuError::NO_RING_SPACE;
    }

    // the message is transferred piece by piece; we can start as soon as
    // we have the header
    Cycles delay = tcu.ticksToCycles(pkt->headerDelay);
    pkt->headerDelay = 0;
    delay += tcu.nocToTransferLatency;

    uint rflags = XferUnit::XferFlags::MSGRECV;
    // receive EPs use a physical address, thus NOPF and NOXLATE.
    rflags |= XferUnit::XferFlags::NOPF | XferUnit::NOXLATE;
    Addr localAddr = ep->r1.bufAddr + (msgidx << ep->r0.msgSize);

    auto *ev = new ReceiveTransferEvent(this, localAddr,
                                        ep->r0.vpe, rflags, pkt);
    tcu.startTransfer(ev, delay);

    return TcuError::NONE;
}

void
MessageUnit::ReceiveTransferEvent::transferDone(TcuError result)
{
    MessageHeader* header = pkt->getPtr<MessageHeader>();
    epid_t epId = rep();

    RecvEp *ep = tcu().regs().getRecvEp(epId);
    if (result == TcuError::NONE && ep != nullptr)
    {
        bool foreign = ep->r0.vpe != tcu().regs().getVPE();
        result = msgUnit->finishMsgReceive(epId, msgAddr, header,
                                           result, flags(), !foreign);

        // notify SW if we received a message for a different VPE
        if(foreign)
            tcu().startForeignReceive(epId, ep->r0.vpe);
    }

    MemoryUnit::ReceiveTransferEvent::transferDone(result);
}
