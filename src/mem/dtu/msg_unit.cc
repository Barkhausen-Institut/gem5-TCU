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
#include "debug/DtuMsgs.hh"
#include "mem/dtu/msg_unit.hh"
#include "mem/dtu/noc_addr.hh"
#include "mem/dtu/xfer_unit.hh"

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

    // if we want to reply, load the reply EP first
    if (cmd.opcode == Dtu::Command::REPLY)
    {
        RecvEp ep = dtu.regs().getRecvEp(epid);
        int msgidx = ep.msgToIdx(cmd.arg);

        if(ep.bufAddr == 0 || ep.vpe != dtu.regs().getVPE())
        {
            DPRINTFS(Dtu, (&dtu), "EP%u: invalid EP\n", epid);
            dtu.scheduleFinishOp(Cycles(1), DtuError::INV_EP);
            return;
        }

        if (ep.replyEps == NO_REPLIES)
        {
            DPRINTFS(Dtu, (&dtu),
                     "EP%u: no reply EPs, cannot reply on msg %p\n",
                     epid, cmd.arg);
            dtu.scheduleFinishOp(Cycles(1), DtuError::INV_EP);
            return;
        }

        epid = ep.replyEps + msgidx;

        SendEp sep = dtu.regs().getSendEp(epid);

        if (sep.maxMsgSize == 0 || !(sep.flags & SendEp::FL_REPLY))
        {
            DPRINTFS(Dtu, (&dtu),
                     "EP%u: invalid reply EP. Double reply for msg %p?\n",
                     epid, cmd.arg);
            dtu.scheduleFinishOp(Cycles(1), DtuError::INV_ARGS);
            return;
        }

        assert(sep.vpe == dtu.regs().getVPE());

        // grant credits to the sender
        info.replyEpId = sep.crdEp;
        info.flags = Dtu::REPLY_FLAG;
        info.replySize = 0;

        // the pagefault flag is moved to the reply hd
        if (sep.flags & SendEp::FL_PF)
            info.flags |= Dtu::PAGEFAULT;
    }

    // check if we have enough credits
    const DataReg data = dtu.regs().getDataReg();
    SendEp ep = dtu.regs().getSendEp(epid);
    RecvEp rep;
    if (cmd.opcode == Dtu::Command::SEND)
        rep = dtu.regs().getRecvEp(cmd.arg);

    if (ep.maxMsgSize == 0 ||
        ep.vpe != dtu.regs().getVPE() ||
        (cmd.opcode == Dtu::Command::SEND && rep.vpe != dtu.regs().getVPE()))
    {
        DPRINTFS(Dtu, (&dtu), "EP%u: invalid EP\n", epid);
        dtu.scheduleFinishOp(Cycles(1), DtuError::INV_EP);
        return;
    }

    if (ep.curcrd != Dtu::CREDITS_UNLIM)
    {
        if (ep.curcrd == 0)
        {
            DPRINTFS(Dtu, (&dtu), "EP%u: no credits to send message\n", epid);
            dtu.scheduleFinishOp(Cycles(1), DtuError::MISS_CREDITS);
            return;
        }

        // pay the credits
        ep.curcrd--;

        DPRINTFS(DtuCredits, (&dtu), "EP%u paid 1 credit (%u left)\n",
                 epid, ep.curcrd);

        dtu.regs().setSendEp(epid, ep);
    }

    // TODO error handling
    assert(data.size + sizeof(MessageHeader) <= (1 << ep.maxMsgSize));

    // fill the info struct and start the transfer
    info.targetCoreId = ep.targetCore;
    info.targetEpId   = ep.targetEp;
    info.label        = ep.label;
    info.replyLabel   = dtu.regs().get(CmdReg::REPLY_LABEL);
    info.unlimcred    = ep.curcrd == Dtu::CREDITS_UNLIM;
    info.ready        = true;

    if (cmd.opcode == Dtu::Command::SEND)
    {
        info.replySize    = rep.msgSize;
        info.replyEpId    = cmd.arg;
        info.flags        = 0;
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
        header->flags = Dtu::REPLY_FLAG;
    else
        header->flags = 0; // normal message
    header->flags |= info.flags;

    header->senderCoreId = dtu.coreId;
    header->senderEpId   = info.unlimcred ? dtu.numEndpoints : cmd.epid;
    header->replyEpId    = info.replyEpId;
    header->length       = data.size;
    header->label        = info.label;
    header->replyLabel   = info.replyLabel;
    header->replySize    = info.replySize;

    DPRINTFS(Dtu, (&dtu),
        "  src: pe=%u ep=%u rpep=%u rplbl=%#018lx rpsize=%#x flags=%#x%s\n",
        header->senderCoreId, header->senderEpId, header->replyEpId,
        header->replyLabel, 1 << header->replySize, header->flags,
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

bool
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
    return true;
}

void
MessageUnit::finishMsgReply(DtuError error, unsigned epid, Addr msgAddr)
{
    ackMessage(epid, msgAddr);
}

void
MessageUnit::finishMsgSend(DtuError error, unsigned epid)
{
    SendEp ep = dtu.regs().getSendEp(epid);
    // don't do anything if the EP is invalid
    if (ep.maxMsgSize == 0 || ep.vpe != dtu.regs().getVPE())
        return;

    // undo the credit reduction on errors except for MISS_CREDITS
    if (ep.curcrd != Dtu::CREDITS_UNLIM &&
        error != DtuError::NONE && error != DtuError::MISS_CREDITS)
    {
        ep.curcrd++;
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
        ep.curcrd++;
        assert(ep.curcrd <= ep.maxcrd);

        DPRINTFS(DtuCredits, (&dtu),
            "EP%u received 1 credit (%u in total)\n",
            epid, ep.curcrd);

        dtu.regs().setSendEp(epid, ep);
    }
}

Addr
MessageUnit::fetchMessage(unsigned epid)
{
    RecvEp ep = dtu.regs().getRecvEp(epid);

    if (ep.unread == 0 || ep.vpe != dtu.regs().getVPE())
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
    ep.rdPos = i + 1;

    DPRINTFS(DtuBuf, (&dtu),
        "EP%u: fetched message at index %u (count=%u)\n",
        epid, i, ep.unreadMsgs());

    dtu.regs().setRecvEp(epid, ep);
    dtu.regs().rem_msg();

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

DtuError
MessageUnit::ackMessage(unsigned epId, Addr msgAddr)
{
    RecvEp ep = dtu.regs().getRecvEp(epId);
    if (ep.bufAddr == 0 || ep.vpe != dtu.regs().getVPE())
        return DtuError::INV_EP;

    int msgidx = ep.msgToIdx(msgAddr);
    if (msgidx == RecvEp::MAX_MSGS || !ep.isOccupied(msgidx))
        return DtuError::INV_MSG;

    ep.setOccupied(msgidx, false);
    if (ep.isUnread(msgidx))
        ep.setUnread(msgidx, false);

    if (ep.replyEps != NO_REPLIES)
    {
        // invalidate reply EP
        dtu.regs().invalidate(ep.replyEps + msgidx, true);
    }

    DPRINTFS(DtuBuf, (&dtu),
        "EP%u: acked msg at index %d\n",
        epId, msgidx);

    dtu.regs().setRecvEp(epId, ep);
    return DtuError::NONE;
}

DtuError
MessageUnit::invalidateReply(unsigned repId, unsigned peId, unsigned sepId)
{
    RecvEp ep = dtu.regs().getRecvEp(repId);
    if (ep.bufAddr == 0 || ep.replyEps == NO_REPLIES)
        return DtuError::INV_EP;

    for (int i = 0; i < (1 << ep.size); ++i)
    {
        auto sep = dtu.regs().getSendEp(ep.replyEps + i);
        if (sep.targetCore == peId && sep.crdEp == sepId)
            dtu.regs().invalidate(ep.replyEps + i, true);
    }
    return DtuError::NONE;
}

DtuError
MessageUnit::finishMsgReceive(unsigned epId,
                              Addr msgAddr,
                              const MessageHeader *header,
                              DtuError error,
                              uint xferFlags,
                              bool addMsg)
{
    RecvEp ep = dtu.regs().getRecvEp(epId);
    if (ep.bufAddr == 0)
        return DtuError::INV_EP;

    int idx = (msgAddr - ep.bufAddr) >> ep.msgSize;

    if (error == DtuError::NONE)
    {
        // Note that replyEpId is the Id of *our* sending EP
        if (header->flags & Dtu::REPLY_FLAG &&
            header->replyEpId < dtu.numEndpoints)
        {
            recvCredits(header->replyEpId);
        }

        DPRINTFS(DtuBuf, (&dtu),
            "EP%u: increment message count to %u\n",
            epId, ep.unreadMsgs() + 1);

        ep.setUnread(idx, true);

        if (!(header->flags & Dtu::REPLY_FLAG))
        {
            assert(ep.replyEps != NO_REPLIES);

            // install use-once reply EP
            SendEp sep;
            sep.targetCore = header->senderCoreId;
            sep.targetEp = header->replyEpId;
            sep.label = header->replyLabel;
            sep.maxMsgSize = header->replySize;
            sep.maxcrd = sep.curcrd = 1;
            sep.crdEp = header->senderEpId;
            sep.flags = SendEp::FL_REPLY;
            sep.vpe = ep.vpe;
            if (header->flags & Dtu::PAGEFAULT)
                sep.flags |= SendEp::FL_PF;
            dtu.regs().setSendEp(ep.replyEps + idx, sep);
        }
    }
    else
        ep.setOccupied(idx, false);

    dtu.regs().setRecvEp(epId, ep);

    if (error == DtuError::NONE && ep.vpe == dtu.regs().getVPE())
    {
        if (addMsg)
            dtu.regs().add_msg();
        dtu.wakeupCore(false);
    }

    return error;
}

DtuError
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
        return DtuError::NONE;
    }

    NocAddr addr(pkt->getAddr());
    unsigned epId = addr.offset;

    DPRINTFS(Dtu, (&dtu),
        "\e[1m[rv <- %u]\e[0m %lu bytes on EP%u\n",
        header->senderCoreId, header->length, epId);
    dtu.printPacket(pkt);

    if (DTRACE(DtuMsgs))
    {
        uint64_t *words = reinterpret_cast<uint64_t*>(header + 1);
        for(size_t i = 0; i < header->length / sizeof(uint64_t); ++i)
            DPRINTFS(DtuMsgs, (&dtu), "    word%2lu: %#018x\n", i, words[i]);
    }

    // support credit receives without storing reply messages
    if (epId >= dtu.numEndpoints &&
        (header->flags & Dtu::REPLY_FLAG) &&
        header->replyEpId < dtu.numEndpoints)
    {
        recvCredits(header->replyEpId);
        dtu.sendNocResponse(pkt);
        dtu.regs().setEvent(EventType::CRD_RECV);
        dtu.wakeupCore(false);
        return DtuError::NONE;
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
        return DtuError::NO_RING_SPACE;
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

    return DtuError::NONE;
}

bool
MessageUnit::ReceiveTransferEvent::transferStart()
{
    MemoryUnit::ReceiveTransferEvent::transferStart();

    NocAddr addr(pkt->getAddr());
    unsigned epId = addr.offset;
    RecvEp ep = dtu().regs().getRecvEp(epId);
    // ignore the error here
    if (ep.bufAddr == 0)
        return true;

    // notify SW if we received a message for a different VPE
    if (ep.vpe != dtu().regs().getVPE())
    {
        coreReq = true;
        dtu().startForeignReceive(bufId(), epId, ep.vpe, this);
        return false;
    }

    return true;
}

void
MessageUnit::ReceiveTransferEvent::transferDone(DtuError result)
{
    MessageHeader* header = pkt->getPtr<MessageHeader>();
    NocAddr addr(pkt->getAddr());

    result = msgUnit->finishMsgReceive(addr.offset, msgAddr, header,
                                       result, flags(), !coreReq);

    MemoryUnit::ReceiveTransferEvent::transferDone(result);
}
