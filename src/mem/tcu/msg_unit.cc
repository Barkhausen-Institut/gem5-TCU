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
MessageUnit::startSend(const CmdCommand::Bits &cmd)
{
    startSendReply(cmd, cmd.epid);
}

void
MessageUnit::startReply(const CmdCommand::Bits &cmd)
{
    epid_t epid = cmd.epid;

    RecvEp *ep = tcu.regs().getRecvEp(epid);

    if(!ep)
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: invalid EP\n", epid);
        tcu.scheduleFinishOp(Cycles(1), TcuError::NO_REP);
        return;
    }

    if(ep->r0.vpe != tcu.regs().getCurVPE().id)
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: foreign EP\n", epid);
        tcu.scheduleFinishOp(Cycles(1), TcuError::FOREIGN_EP);
        return;
    }

    if (ep->r0.rplEps == Tcu::INVALID_EP_ID)
    {
        DPRINTFS(Tcu, (&tcu),
                 "EP%u: no reply EPs, cannot reply on msg %p\n",
                 epid, cmd.arg0);
        tcu.scheduleFinishOp(Cycles(1), TcuError::REPLIES_DISABLED);
        return;
    }

    int msgidx = ep->offsetToIdx(cmd.arg0);
    if (msgidx == RecvEp::MAX_MSGS)
    {
        DPRINTFS(Tcu, (&tcu),
                 "EP%u: offset out of bounds (%#x)\n", epid, cmd.arg0);
        tcu.scheduleFinishOp(Cycles(1), TcuError::INV_MSG_OFF);
        return;
    }

    epid_t sepid = ep->r0.rplEps + msgidx;
    startSendReply(cmd, sepid);
}

void
MessageUnit::startSendReply(const CmdCommand::Bits &cmd, epid_t epid)
{
    cmdSep = epid;

    const CmdData::Bits data = tcu.regs().getData();
    SendEp *ep = tcu.regs().getSendEp(epid);

    // check if the send EP is valid
    if(!ep)
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: invalid EP\n", epid);
        tcu.scheduleFinishOp(Cycles(1), TcuError::NO_SEP);
        return;
    }

    if(ep->r0.vpe != tcu.regs().getCurVPE().id)
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: foreign EP\n", epid);
        tcu.scheduleFinishOp(Cycles(1), TcuError::FOREIGN_EP);
        return;
    }

    if ((cmd.opcode == CmdCommand::SEND && ep->r0.reply) ||
        (cmd.opcode == CmdCommand::REPLY && !ep->r0.reply))
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: send vs. reply\n", epid);
        tcu.scheduleFinishOp(Cycles(1), TcuError::SEND_REPLY_EP);
        return;
    }

    // check message size
    if (data.size + sizeof(MessageHeader) > (1 << ep->r0.msgSize))
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: message too large\n", epid);
        tcu.scheduleFinishOp(Cycles(1), TcuError::OUT_OF_BOUNDS);
        return;
    }

    epid_t replyEpId;
    size_t replySize;

    // get info from receive EP
    if (cmd.opcode == CmdCommand::SEND)
    {
        if (cmd.arg0 != Tcu::INVALID_EP_ID)
        {
            RecvEp *rep = tcu.regs().getRecvEp(cmd.arg0);
            if(!rep)
            {
                DPRINTFS(Tcu, (&tcu), "EP%u: invalid EP\n", cmd.arg0);
                tcu.scheduleFinishOp(Cycles(1), TcuError::NO_REP);
                return;
            }

            if(rep->r0.vpe != tcu.regs().getCurVPE().id)
            {
                DPRINTFS(Tcu, (&tcu), "EP%u: foreign EP\n", cmd.arg0);
                tcu.scheduleFinishOp(Cycles(1), TcuError::FOREIGN_EP);
                return;
            }

            replyEpId = cmd.arg0;
            replySize = rep->r0.slotSize;
        }
        else
        {
            replyEpId = Tcu::INVALID_EP_ID;
            replySize = ceil(log2(sizeof(MessageHeader)));
        }
    }
    else
    {
        assert(ep->r0.vpe == tcu.regs().getCurVPE().id);

        // grant credits to the sender
        replyEpId = ep->r0.crdEp;
        replySize = 0;
    }

    if (cmd.opcode == CmdCommand::REPLY)
        repliedBytes.sample(data.size);
    else
        sentBytes.sample(data.size);

    DPRINTFS(Tcu, (&tcu), "\e[1m[%s -> %u]\e[0m with EP%u of %#018lx:%lu\n",
             cmd.opcode == CmdCommand::REPLY ? "rp" : "sd",
             ep->r1.tgtPe,
             cmd.epid,
             data.addr,
             data.size);

    // build header
    MessageHeader* header = new MessageHeader;
    if (cmd.opcode == CmdCommand::REPLY)
        header->flags = Tcu::REPLY_FLAG;
    else
        header->flags = 0; // normal message

    header->senderPeId   = tcu.peId;
    header->senderEpId   = ep->r0.curCrd == Tcu::CREDITS_UNLIM
                           ? Tcu::INVALID_EP_ID
                           : cmd.epid;
    header->replyEpId    = replyEpId;
    header->length       = data.size;
    header->label        = ep->r2.label;
    header->replyLabel   = tcu.regs().get(UnprivReg::ARG1);
    header->replySize    = replySize;

    DPRINTFS(Tcu, (&tcu),
        "  src: pe=%u ep=%u rpep=%u rplbl=%#018lx rpsize=%#x flags=%#x%s\n",
        header->senderPeId, header->senderEpId, header->replyEpId,
        header->replyLabel, 1 << header->replySize, header->flags,
        header->senderPeId != tcu.peId ? " (on behalf)" : "");

    DPRINTFS(Tcu, (&tcu),
        "  dst: pe=%u ep=%u lbl=%#018lx\n",
        ep->r1.tgtPe, ep->r1.tgtEp, header->label);

    assert(data.size + sizeof(MessageHeader) <= tcu.maxNocPacketSize);

    NocAddr nocAddr(ep->r1.tgtPe, ep->r1.tgtEp);
    uint flags = XferUnit::MESSAGE;

    // start the transfer of the payload
    auto *ev = new SendTransferEvent(
        this, data.addr, data.size, flags, nocAddr, header);
    tcu.startTransfer(ev, tcu.startMsgTransferDelay);
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
MessageUnit::SendTransferEvent::transferDone(TcuError result)
{
    if (result == TcuError::NONE)
    {
        CmdCommand::Bits cmd = tcu().regs().getCommand();

        if (cmd.opcode == CmdCommand::REPLY)
            msgUnit->ackMessage(cmd.epid, cmd.arg0);
        else
        {
            // check if we have enough credits
            SendEp *ep = tcu().regs().getSendEp(msgUnit->cmdSep);
            if (ep->r0.curCrd != Tcu::CREDITS_UNLIM)
            {
                if (ep->r0.curCrd == 0)
                {
                    DPRINTFS(Tcu, (&tcu()),
                             "EP%u: no credits to send message\n",
                             msgUnit->cmdSep);
                    result = TcuError::NO_CREDITS;
                }
                else
                {
                    // pay the credits
                    ep->r0.curCrd = ep->r0.curCrd - 1;

                    DPRINTFS(TcuCredits, (&tcu()),
                             "EP%u paid 1 credit (%u left)\n",
                             msgUnit->cmdSep, ep->r0.curCrd);

                    tcu().regs().updateEp(msgUnit->cmdSep);
                }
            }
        }
    }

    MemoryUnit::WriteTransferEvent::transferDone(result);
}

void
MessageUnit::recvCredits(epid_t epid)
{
    SendEp *ep = tcu.regs().getSendEp(epid);
    // don't do anything if the EP is invalid
    if (!ep)
        return;

    if (ep->r0.curCrd != Tcu::CREDITS_UNLIM)
    {
        ep->r0.curCrd = ep->r0.curCrd + 1;
        assert(ep->r0.curCrd <= ep->r0.maxCrd);

        DPRINTFS(TcuCredits, (&tcu),
            "EP%u received 1 credit (%u in total)\n",
            epid, ep->r0.curCrd);

        tcu.regs().updateEp(epid);
    }
}

TcuError
MessageUnit::fetchMessage(epid_t epid, Addr *msgOff)
{
    RecvEp *ep = tcu.regs().getRecvEp(epid);
    if (!ep)
        return TcuError::NO_REP;
    if (ep->r0.vpe != tcu.regs().getCurVPE().id)
        return TcuError::FOREIGN_EP;

    // check if the current VPE has unread messages at all. note that this is
    // important in case it is out of sync with the receive EPs, i.e., if we
    // have ongoing foreignRecv core requests.
    if (ep->r2.unread == 0 || tcu.regs().getCurVPE().msgs == 0)
    {
        *msgOff = -1;
        return TcuError::NONE;
    }

    int i;
    for (i = ep->r0.rpos; i < (1 << ep->r0.slots); ++i)
    {
        if (ep->isUnread(i))
            goto found;
    }
    for (i = 0; i < ep->r0.rpos; ++i)
    {
        if (ep->isUnread(i))
            goto found;
    }

    // should not get here
    assert(false);

found:
    assert(ep->isOccupied(i));

    ep->setUnread(i, false);
    ep->r0.rpos = i + 1;

    DPRINTFS(TcuBuf, (&tcu),
        "EP%u: fetched message at index %u (count=%u)\n",
        epid, i, ep->unreadMsgs());

    tcu.regs().updateEp(epid);
    tcu.regs().rem_msg();

    *msgOff = i << ep->r0.slotSize;
    return TcuError::NONE;
}

int
MessageUnit::allocSlot(size_t msgSize, epid_t epid, RecvEp *ep)
{
    assert(ep != nullptr);
    assert(msgSize <= (1 << ep->r0.slotSize));

    int i;
    for (i = ep->r0.wpos; i < (1 << ep->r0.slots); ++i)
    {
        if (!ep->isOccupied(i))
            goto found;
    }
    for (i = 0; i < ep->r0.wpos; ++i)
    {
        if (!ep->isOccupied(i))
            goto found;
    }

    return -1;

found:
    ep->setOccupied(i, true);
    ep->r0.wpos = i + 1;

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
    if (!ep)
        return TcuError::NO_REP;
    if (ep->r0.vpe != tcu.regs().getCurVPE().id)
        return TcuError::FOREIGN_EP;

    int msgidx = ep->offsetToIdx(msgOff);
    if (msgidx == RecvEp::MAX_MSGS)
        return TcuError::INV_MSG_OFF;

    bool unread = false;
    ep->setOccupied(msgidx, false);
    if (ep->isUnread(msgidx))
    {
        ep->setUnread(msgidx, false);
        unread = true;
    }

    if (ep->r0.rplEps != Tcu::INVALID_EP_ID)
    {
        // invalidate reply EP
        unsigned unread_mask;
        tcu.regs().invalidate(ep->r0.rplEps + msgidx, true, &unread_mask);
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
MessageUnit::finishMsgReceive(epid_t epId,
                              Addr msgAddr,
                              const MessageHeader *header,
                              TcuError error,
                              uint xferFlags,
                              bool addMsg)
{
    RecvEp *ep = tcu.regs().getRecvEp(epId);
    if (!ep)
        return TcuError::NO_REP;

    int idx = (msgAddr - ep->r1.buffer) >> ep->r0.slotSize;

    if (error == TcuError::NONE)
    {
        DPRINTFS(TcuBuf, (&tcu),
            "EP%u: increment message count to %u\n",
            epId, ep->unreadMsgs() + 1);

        ep->setUnread(idx, true);

        if (!(header->flags & Tcu::REPLY_FLAG) &&
            ep->r0.rplEps != Tcu::INVALID_EP_ID &&
            header->replyEpId != Tcu::INVALID_EP_ID)
        {
            // install use-once reply EP
            Ep *sep = tcu.regs().getEp(ep->r0.rplEps + idx);
            sep->send.r0.type = static_cast<uint>(EpType::SEND);
            sep->send.r0.vpe = ep->r0.vpe;
            sep->send.r0.msgSize = header->replySize;
            sep->send.r0.maxCrd = sep->send.r0.curCrd = 1;
            sep->send.r0.crdEp = header->senderEpId;
            sep->send.r0.reply = 1;
            sep->send.r1.tgtPe = header->senderPeId;
            sep->send.r1.tgtEp = header->replyEpId;
            sep->send.r2.label = header->replyLabel;
            tcu.regs().updateEp(ep->r0.rplEps + idx);
        }

        // Note that replyEpId is the Id of *our* sending EP
        if (header->flags & Tcu::REPLY_FLAG &&
            header->replyEpId != Tcu::INVALID_EP_ID)
        {
            recvCredits(header->replyEpId);
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
    if (!ep || (ep->r1.buffer & 0x7) != 0)
    {
        DPRINTFS(Tcu, (&tcu),
            "EP%u: ignoring message: receive EP invalid\n",
            epId);
        tcu.sendNocResponse(pkt);
        return !ep ? TcuError::RECV_GONE : TcuError::RECV_MISALIGN;
    }

    int msgidx = allocSlot(pkt->getSize(), epId, ep);
    if (msgidx == -1)
    {
        DPRINTFS(Tcu, (&tcu),
            "EP%u: ignoring message: no space left\n",
            epId);
        noSpace++;

        tcu.sendNocResponse(pkt);
        return TcuError::RECV_NO_SPACE;
    }

    // the message is transferred piece by piece; we can start as soon as
    // we have the header
    Cycles delay = tcu.ticksToCycles(pkt->headerDelay);
    pkt->headerDelay = 0;
    delay += tcu.nocToTransferLatency;

    uint rflags = XferUnit::XferFlags::MSGRECV;
    // receive EPs use a physical address, thus NOPF and NOXLATE.
    rflags |= XferUnit::XferFlags::NOPF | XferUnit::NOXLATE;
    Addr localAddr = ep->r1.buffer + (msgidx << ep->r0.slotSize);

    auto *ev = new ReceiveTransferEvent(this, localAddr, rflags, pkt);
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
        bool foreign = ep->r0.vpe != tcu().regs().getCurVPE().id;
        result = msgUnit->finishMsgReceive(epId, msgAddr, header,
                                           result, flags(), !foreign);

        // notify SW if we received a message for a different VPE
        if(foreign)
            tcu().startForeignReceive(epId, ep->r0.vpe);
    }

    MemoryUnit::ReceiveTransferEvent::transferDone(result);
}
