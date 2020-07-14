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
    cmdEps.addEp(cmd.epid);
    if (cmd.arg0 != Tcu::INVALID_EP_ID)
        cmdEps.addEp(cmd.arg0);
    cmdEps.onFetched(std::bind(&MessageUnit::startSendReplyWithEP,
                               this, std::placeholders::_1, cmd.epid));
}

void
MessageUnit::startReply(const CmdCommand::Bits &cmd)
{
    cmdEps.addEp(cmd.epid);
    cmdEps.onFetched(std::bind(&MessageUnit::startReplyWithEP,
                               this, std::placeholders::_1));
}

void
MessageUnit::startReplyWithEP(EpFile::EpCache &eps)
{
    const CmdCommand::Bits cmd = tcu.regs().getCommand();

    const Ep ep = eps.getEp(cmd.epid);

    if(ep.type() != EpType::RECEIVE)
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: invalid EP\n", cmd.epid);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NO_REP);
        return;
    }

    const RecvEp &rep = ep.recv;

    if(rep.r0.vpe != tcu.regs().getCurVPE().id)
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: foreign EP\n", cmd.epid);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::FOREIGN_EP);
        return;
    }

    if (rep.r0.rplEps == Tcu::INVALID_EP_ID)
    {
        DPRINTFS(Tcu, (&tcu),
                 "EP%u: no reply EPs, cannot reply on msg %p\n",
                 cmd.epid, cmd.arg0);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::REPLIES_DISABLED);
        return;
    }

    int msgidx = rep.offsetToIdx(cmd.arg0);
    if (msgidx == RecvEp::MAX_MSGS)
    {
        DPRINTFS(Tcu, (&tcu),
                 "EP%u: offset out of bounds (%#x)\n", cmd.epid, cmd.arg0);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::INV_MSG_OFF);
        return;
    }

    epid_t sepid = rep.r0.rplEps + msgidx;
    eps.addEp(sepid);
    eps.onFetched(std::bind(&MessageUnit::startSendReplyWithEP,
                            this, std::placeholders::_1, sepid));
}

void
MessageUnit::startSendReplyWithEP(EpFile::EpCache &eps, epid_t epid)
{
    const CmdCommand::Bits cmd = tcu.regs().getCommand();
    const CmdData::Bits data = tcu.regs().getData();

    const Ep ep = eps.getEp(epid);

    Ep replyEp(0);
    if (cmd.opcode == CmdCommand::SEND && cmd.arg0 != Tcu::INVALID_EP_ID)
        replyEp = eps.getEp(cmd.arg0);

    if(ep.type() != EpType::SEND)
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: invalid EP\n", epid);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NO_SEP);
        return;
    }

    const SendEp &sep = ep.send;

    if(sep.r0.vpe != tcu.regs().getCurVPE().id)
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: foreign EP\n", epid);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::FOREIGN_EP);
        return;
    }

    if ((cmd.opcode == CmdCommand::SEND && sep.r0.reply) ||
        (cmd.opcode == CmdCommand::REPLY && !sep.r0.reply))
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: send vs. reply\n", epid);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::SEND_REPLY_EP);
        return;
    }

    // check message size
    if (data.size + sizeof(MessageHeader) > (1 << sep.r0.msgSize))
    {
        DPRINTFS(Tcu, (&tcu), "EP%u: message too large\n", epid);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::OUT_OF_BOUNDS);
        return;
    }

    epid_t replyEpId;
    size_t replySize;

    // get info from receive EP
    if (cmd.opcode == CmdCommand::SEND)
    {
        if (cmd.arg0 != Tcu::INVALID_EP_ID)
        {
            if(replyEp.type() != EpType::RECEIVE)
            {
                DPRINTFS(Tcu, (&tcu), "EP%u: invalid EP\n", cmd.arg0);
                tcu.scheduleCmdFinish(Cycles(1), TcuError::NO_REP);
                return;
            }

            const RecvEp &rep = replyEp.recv;

            if(rep.r0.vpe != tcu.regs().getCurVPE().id)
            {
                DPRINTFS(Tcu, (&tcu), "EP%u: foreign EP\n", cmd.arg0);
                tcu.scheduleCmdFinish(Cycles(1), TcuError::FOREIGN_EP);
                return;
            }

            replyEpId = cmd.arg0;
            replySize = rep.r0.slotSize;
        }
        else
        {
            replyEpId = Tcu::INVALID_EP_ID;
            replySize = ceil(log2(sizeof(MessageHeader)));
        }
    }
    else
    {
        assert(sep.r0.vpe == tcu.regs().getCurVPE().id);

        // grant credits to the sender
        replyEpId = sep.r0.crdEp;
        replySize = 0;
    }

    if (cmd.opcode == CmdCommand::REPLY)
        repliedBytes.sample(data.size);
    else
        sentBytes.sample(data.size);

    DPRINTFS(Tcu, (&tcu), "\e[1m[%s -> %u]\e[0m with EP%u of %#018lx:%lu\n",
             cmd.opcode == CmdCommand::REPLY ? "rp" : "sd",
             sep.r1.tgtPe,
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
    header->senderEpId   = sep.r0.curCrd == Tcu::CREDITS_UNLIM
                           ? Tcu::INVALID_EP_ID
                           : cmd.epid;
    header->replyEpId    = replyEpId;
    header->length       = data.size;
    header->label        = sep.r2.label;
    header->replyLabel   = tcu.regs().get(UnprivReg::ARG1);
    header->replySize    = replySize;

    DPRINTFS(Tcu, (&tcu),
        "  src: pe=%u ep=%u rpep=%u rplbl=%#018lx rpsize=%#x flags=%#x%s\n",
        header->senderPeId, header->senderEpId, header->replyEpId,
        header->replyLabel, 1 << header->replySize, header->flags,
        header->senderPeId != tcu.peId ? " (on behalf)" : "");

    DPRINTFS(Tcu, (&tcu),
        "  dst: pe=%u ep=%u lbl=%#018lx\n",
        sep.r1.tgtPe, sep.r1.tgtEp, header->label);

    assert(data.size + sizeof(MessageHeader) <= tcu.maxNocPacketSize);

    NocAddr nocAddr(sep.r1.tgtPe, sep.r1.tgtEp);
    uint flags = XferUnit::MESSAGE;

    // start the transfer of the payload
    auto *ev = new SendTransferEvent(
        this, sep.id, data.addr, data.size, flags, nocAddr, header);
    tcu.startTransfer(ev, tcu.startMsgTransferDelay);

    eps.setAutoFinish(false);
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
        {
            RecvEp rep = msgUnit->cmdEps.getEp(cmd.epid).recv;
            int msgidx = rep.offsetToIdx(cmd.arg0);
            msgUnit->ackMessage(rep, msgidx);
        }
        else
        {
            SendEp sep = msgUnit->cmdEps.getEp(sepid).send;
            // check if we have enough credits
            if (sep.r0.curCrd != Tcu::CREDITS_UNLIM)
            {
                if (sep.r0.curCrd == 0)
                {
                    DPRINTFS(Tcu, (&tcu()),
                             "EP%u: no credits to send message\n",
                             sep.id);
                    result = TcuError::NO_CREDITS;
                }
                else
                {
                    // pay the credits
                    sep.r0.curCrd = sep.r0.curCrd - 1;

                    DPRINTFS(TcuCredits, (&tcu()),
                             "EP%u paid 1 credit (%u left)\n",
                             sep.id, sep.r0.curCrd);

                    msgUnit->cmdEps.updateEp(sep);
                }
            }
        }
    }

    MemoryUnit::WriteTransferEvent::transferDone(result);

    msgUnit->cmdEps.onFinished([](EpFile::EpCache &) {});
}

void
MessageUnit::startInvalidate(const ExtCommand::Bits &cmd)
{
    extCmdEps.addEp(cmd.arg & 0xFFFF);
    extCmdEps.onFetched(std::bind(&MessageUnit::invalidateWithEP,
                                  this, std::placeholders::_1));
}

void
MessageUnit::invalidateWithEP(EpFile::EpCache &eps)
{
    ExtCommand::Bits cmd = tcu.regs().get(ExtReg::EXT_CMD);

    epid_t epid = cmd.arg & 0xFFFF;
    bool force = !!(cmd.arg & (1 << 16));
    unsigned unreadMask = 0;

    Ep ep = eps.getEp(epid);

    if (!force && ep.type() == EpType::SEND)
    {
        if (ep.send.r0.curCrd != ep.send.r0.maxCrd)
        {
            tcu.scheduleExtCmdFinish(Cycles(1), TcuError::NO_CREDITS, 0);
            return;
        }
    }

    if (!force && ep.type() == EpType::RECEIVE)
        unreadMask = ep.recv.r2.unread;

    for (int i = 0; i < numEpRegs; ++i)
        ep.inval.r[i] = 0;
    eps.updateEp(ep.send);

    eps.onFinished([this, unreadMask](EpFile::EpCache &) {
        tcu.scheduleExtCmdFinish(Cycles(1), TcuError::NONE, unreadMask);
    });
}

void
MessageUnit::startFetch(const CmdCommand::Bits &cmd)
{
    cmdEps.addEp(cmd.epid);
    cmdEps.onFetched(std::bind(&MessageUnit::fetchWithEP,
                               this, std::placeholders::_1));
}

void
MessageUnit::fetchWithEP(EpFile::EpCache &eps)
{
    CmdCommand::Bits cmd = tcu.regs().getCommand();

    Ep ep = eps.getEp(cmd.epid);
    if (ep.type() != EpType::RECEIVE)
    {
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NO_REP);
        return;
    }

    RecvEp &rep = ep.recv;
    if (rep.r0.vpe != tcu.regs().getCurVPE().id)
    {
        tcu.scheduleCmdFinish(Cycles(1), TcuError::FOREIGN_EP);
        return;
    }

    // check if the current VPE has unread messages at all. note that this is
    // important in case it is out of sync with the receive EPs, i.e., if we
    // have ongoing foreignRecv core requests.
    if (rep.r2.unread == 0 || tcu.regs().getCurVPE().msgs == 0)
    {
        tcu.regs().set(UnprivReg::ARG1, -1);
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
        return;
    }

    int i;
    for (i = rep.r0.rpos; i < (1 << rep.r0.slots); ++i)
    {
        if (rep.isUnread(i))
            goto found;
    }
    for (i = 0; i < rep.r0.rpos; ++i)
    {
        if (rep.isUnread(i))
            goto found;
    }

    // should not get here
    assert(false);

found:
    assert(rep.isOccupied(i));

    rep.setUnread(i, false);
    rep.r0.rpos = i + 1;

    DPRINTFS(TcuBuf, (&tcu),
        "EP%u: fetched message at index %u (count=%u)\n",
        cmd.epid, i, rep.unreadMsgs());

    eps.updateEp(rep);
    tcu.regs().rem_msg();
    tcu.regs().set(UnprivReg::ARG1, i << rep.r0.slotSize);

    eps.onFinished([this](EpFile::EpCache &) {
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
    });
}

void
MessageUnit::startAck(const CmdCommand::Bits &cmd)
{
    cmdEps.addEp(cmd.epid);
    cmdEps.onFetched(std::bind(&MessageUnit::startAckWithEP,
                     this, std::placeholders::_1));
}

void
MessageUnit::startAckWithEP(EpFile::EpCache &eps)
{
    CmdCommand::Bits cmd = tcu.regs().getCommand();

    Ep ep = eps.getEp(cmd.epid);
    if (ep.type() != EpType::RECEIVE)
    {
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NO_REP);
        return;
    }

    RecvEp &rep = ep.recv;
    if (rep.r0.vpe != tcu.regs().getCurVPE().id)
    {
        tcu.scheduleCmdFinish(Cycles(1), TcuError::FOREIGN_EP);
        return;
    }

    int msgidx = rep.offsetToIdx(cmd.arg0);
    if (msgidx == RecvEp::MAX_MSGS)
    {
        tcu.scheduleCmdFinish(Cycles(1), TcuError::INV_MSG_OFF);
        return;
    }

    ackMessage(rep, msgidx);

    eps.onFinished([this](EpFile::EpCache &) {
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
    });
}

void MessageUnit::ackMessage(RecvEp &rep, int msgidx)
{
    bool unread = false;
    rep.setOccupied(msgidx, false);
    if (rep.isUnread(msgidx))
    {
        rep.setUnread(msgidx, false);
        unread = true;
    }

    if (rep.r0.rplEps != Tcu::INVALID_EP_ID)
    {
        // invalidate reply EP
        SendEp replyEp;
        replyEp.id = rep.r0.rplEps + msgidx;
        replyEp.r0 = 0;
        replyEp.r1 = 0;
        replyEp.r2 = 0;
        cmdEps.updateEp(replyEp);
    }

    DPRINTFS(TcuBuf, (&tcu),
        "EP%u: acked msg at index %d\n",
        rep.id, msgidx);

    cmdEps.updateEp(rep);
    if (unread)
        tcu.regs().rem_msg();
}

int
MessageUnit::allocSlot(EpFile::EpCache &eps, size_t msgSize, RecvEp &ep)
{
    assert(msgSize <= (1 << ep.r0.slotSize));

    int i;
    for (i = ep.r0.wpos; i < (1 << ep.r0.slots); ++i)
    {
        if (!ep.isOccupied(i))
            goto found;
    }
    for (i = 0; i < ep.r0.wpos; ++i)
    {
        if (!ep.isOccupied(i))
            goto found;
    }

    return -1;

found:
    ep.setOccupied(i, true);
    ep.r0.wpos = i + 1;

    DPRINTFS(TcuBuf, (&tcu),
        "EP%u: put message at index %u\n",
        ep.id, i);

    eps.updateEp(ep);
    return i;
}

void
MessageUnit::recvCredits(EpFile::EpCache &eps, SendEp &sep)
{
    if (sep.r0.curCrd != Tcu::CREDITS_UNLIM)
    {
        sep.r0.curCrd = sep.r0.curCrd + 1;
        assert(sep.r0.curCrd <= sep.r0.maxCrd);

        DPRINTFS(TcuCredits, (&tcu),
            "EP%u received 1 credit (%u in total)\n",
            sep.id, sep.r0.curCrd);

        eps.updateEp(sep);
    }
}

TcuError
MessageUnit::finishMsgReceive(EpFile::EpCache &eps,
                              RecvEp &ep,
                              Addr msgAddr,
                              const MessageHeader *header,
                              TcuError error,
                              uint xferFlags,
                              bool addMsg)
{
    int idx = (msgAddr - ep.r1.buffer) >> ep.r0.slotSize;

    if (error == TcuError::NONE)
    {
        DPRINTFS(TcuBuf, (&tcu),
            "EP%u: increment message count to %u\n",
            ep.id, ep.unreadMsgs() + 1);

        ep.setUnread(idx, true);

        if (!(header->flags & Tcu::REPLY_FLAG) &&
            ep.r0.rplEps != Tcu::INVALID_EP_ID &&
            header->replyEpId != Tcu::INVALID_EP_ID)
        {
            // install use-once reply EP
            SendEp rep;
            rep.id = ep.r0.rplEps + idx;
            rep.r0.type = static_cast<uint>(EpType::SEND);
            rep.r0.vpe = ep.r0.vpe;
            rep.r0.msgSize = header->replySize;
            rep.r0.maxCrd = rep.r0.curCrd = 1;
            rep.r0.crdEp = header->senderEpId;
            rep.r0.reply = 1;
            rep.r1.tgtPe = header->senderPeId;
            rep.r1.tgtEp = header->replyEpId;
            rep.r2.label = header->replyLabel;
            eps.updateEp(rep);
        }

        if (header->flags & Tcu::REPLY_FLAG &&
            header->replyEpId != Tcu::INVALID_EP_ID)
        {
            Ep sep = eps.getEp(header->replyEpId);
            if (sep.type() == EpType::SEND)
                recvCredits(eps, sep.send);
        }
    }
    else
        ep.setOccupied(idx, false);

    eps.updateEp(ep);

    if (error == TcuError::NONE && addMsg)
    {
        tcu.regs().add_msg();
        tcu.wakeupCore(false, ep.id);
    }

    return error;
}

void
MessageUnit::recvFromNoc(PacketPtr pkt)
{
    assert(pkt->isWrite());
    assert(pkt->hasData());

    MessageHeader *header = pkt->getPtr<MessageHeader>();

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

    EpFile::EpCache *cache = new EpFile::EpCache(tcu.eps().newCache());
    cache->addEp(epId);
    // Note that replyEpId is the Id of *our* sending EP
    if (header->flags & Tcu::REPLY_FLAG &&
        header->replyEpId != Tcu::INVALID_EP_ID)
        cache->addEp(header->replyEpId);
    cache->onFetched(std::bind(&MessageUnit::recvFromNocWithEP,
                       this, std::placeholders::_1, pkt));
}

void
MessageUnit::recvFromNocWithEP(EpFile::EpCache &eps, PacketPtr pkt)
{
    NocAddr addr(pkt->getAddr());
    epid_t epid = addr.offset;

    Ep ep = eps.getEp(epid);

    if (ep.type() != EpType::RECEIVE)
    {
        DPRINTFS(Tcu, (&tcu),
            "EP%u: ignoring message: receive EP invalid\n",
            epid);
        tcu.sendNocResponse(pkt, TcuError::RECV_GONE);
        return;
    }

    RecvEp &rep = ep.recv;

    if ((rep.r1.buffer & 0x7) != 0)
    {
        DPRINTFS(Tcu, (&tcu),
            "EP%u: ignoring message: receive EP invalid\n",
            epid);
        tcu.sendNocResponse(pkt, TcuError::RECV_MISALIGN);
        return;
    }

    int msgidx = allocSlot(eps, pkt->getSize(), rep);
    if (msgidx == -1)
    {
        DPRINTFS(Tcu, (&tcu),
            "EP%u: ignoring message: no space left\n",
            epid);
        noSpace++;

        tcu.sendNocResponse(pkt, TcuError::RECV_NO_SPACE);
        return;
    }

    // the message is transferred piece by piece; we can start as soon as
    // we have the header
    Cycles delay = tcu.ticksToCycles(pkt->headerDelay);
    pkt->headerDelay = 0;
    delay += tcu.nocToTransferLatency;

    uint rflags = XferUnit::XferFlags::MSGRECV;
    // receive EPs use a physical address, thus NOPF and NOXLATE.
    rflags |= XferUnit::XferFlags::NOPF | XferUnit::NOXLATE;
    Addr localAddr = rep.r1.buffer + (msgidx << rep.r0.slotSize);

    auto *ev = new ReceiveTransferEvent(
        this, &eps, epid, localAddr, rflags, pkt);
    tcu.startTransfer(ev, delay);

    eps.setAutoFinish(false);
}

void
MessageUnit::ReceiveTransferEvent::transferDone(TcuError result)
{
    MessageHeader *header = pkt->getPtr<MessageHeader>();

    if (result == TcuError::NONE)
    {
        RecvEp rep = eps->getEp(epid).recv;

        bool foreign = rep.r0.vpe != tcu().regs().getCurVPE().id;
        result = msgUnit->finishMsgReceive(*eps, rep, msgAddr, header,
                                           result, flags(), !foreign);

        // notify SW if we received a message for a different VPE
        if(foreign)
            tcu().startForeignReceive(rep.id, rep.r0.vpe);
    }

    MemoryUnit::ReceiveTransferEvent::transferDone(result);

    eps->onFinished([](EpFile::EpCache &eps) {
        delete &eps;
    });
}
