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
#include "debug/DtuDetail.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuSysCalls.hh"
#include "debug/DtuPower.hh"
#include "mem/dtu/msg_unit.hh"

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
    Addr messageAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr messageSize = dtu.regs().get(CmdReg::DATA_SIZE);
    unsigned credits = dtu.regs().get(cmd.epId, EpReg::CREDITS);

    /*
     * If this endpoint is configured to send messages, we need to check
     * credits. If it is configured to receive messages we do a reply and don't
     * need to check credits.
     */
    if (cmd.opcode == Dtu::CommandOpcode::SEND)
    {
        unsigned maxMessageSize = dtu.regs().get(cmd.epId, EpReg::MAX_MSG_SIZE);

        // TODO error handling
        // TODO atm, we can send (nearly) arbitrary large replies
        assert(messageSize + sizeof(Dtu::MessageHeader) <= maxMessageSize);

        if (credits < maxMessageSize)
        {
            warn("pe%u.ep%u: Ignore send message command because there are not "
                 "enough credits", dtu.coreId, cmd.epId);
            dtu.scheduleFinishOp(Cycles(1));
            return;
        }

        DPRINTF(DtuDetail, "EP%u pays %u credits\n", cmd.epId, maxMessageSize);

        // Pay some credits
        credits -= maxMessageSize;
        dtu.regs().set(cmd.epId, EpReg::CREDITS, credits);
    }

    DPRINTF(DtuDetail, "Read message of %lu Bytes at address %#018lx from local scratchpad.\n",
                 messageSize,
                 messageAddr);

    // TODO error handling
    assert(messageSize > 0);
    assert(messageSize + sizeof(Dtu::MessageHeader) <= dtu.maxNocPacketSize);

    auto pkt = dtu.generateRequest(messageAddr, messageSize, MemCmd::ReadReq);

    dtu.sendSpmRequest(pkt,
                       cmd.epId,
                       dtu.commandToSpmRequestLatency,
                       Dtu::SpmPacketType::LOCAL_REQUEST);
}

void
MessageUnit::sendToNoc(const uint8_t* data,
                            Addr messageSize,
                            bool isReply,
                            Tick spmPktHeaderDelay,
                            Tick spmPktPayloadDelay)
{
    unsigned epid = dtu.getCommand().epId;

    unsigned targetCoreId;
    unsigned targetEpId;
    unsigned replyEpId;
    uint64_t label;
    uint64_t replyLabel;

    if (isReply)
    {
        /*
         * We need to read the header of the received message from scratchpad
         * to determine target core and enspoint ID. This would introduce a
         * second scratchpad request and would make the control flow more
         * complicated. To simplify things a functional request is used and an
         * additional delay is payed.
         */

        auto pkt = dtu.generateRequest(
                dtu.translate(dtu.regs().get(epid, EpReg::BUF_RD_PTR)),
                sizeof(Dtu::MessageHeader),
                MemCmd::ReadReq);

        dtu.sendFunctionalSpmRequest(pkt);

        auto h = pkt->getPtr<Dtu::MessageHeader>();
        assert(h->flags & Dtu::REPLY_ENABLED);

        targetCoreId = h->senderCoreId;
        targetEpId   = h->replyEpId;  // send message to the reply EP
        replyEpId    = h->senderEpId; // and grant credits to the sender

        // the receiver of the reply should get the label that he has set
        label        = h->replyLabel;
        // replies don't have replies. so, we don't need that
        replyLabel   = 0;

        // disable replies for this message
        auto hpkt = dtu.generateRequest(
                dtu.translate(dtu.regs().get(epid, EpReg::BUF_RD_PTR)),
                sizeof(h->flags), MemCmd::WriteReq);
        h->flags &= ~Dtu::REPLY_ENABLED;
        memcpy(hpkt->getPtr<uint8_t>(), &h->flags, sizeof(h->flags));

        dtu.sendFunctionalSpmRequest(hpkt);

        dtu.freeRequest(hpkt);
        dtu.freeRequest(pkt);
    }
    else
    {
        targetCoreId = dtu.regs().get(epid, EpReg::TGT_COREID);
        targetEpId   = dtu.regs().get(epid, EpReg::TGT_EPID);
        label        = dtu.regs().get(epid, EpReg::LABEL);
        replyLabel   = dtu.regs().get(CmdReg::REPLY_LABEL);
        replyEpId    = dtu.regs().get(CmdReg::REPLY_EPID);

        M5_VAR_USED unsigned maxMessageSize = dtu.regs().get(epid, EpReg::MAX_MSG_SIZE);
        assert(messageSize + sizeof(Dtu::MessageHeader) <= maxMessageSize);
    }

    DPRINTF(Dtu, "\e[1m[%s -> %u]\e[0m with EP%u of %#018lx:%lu\n",
        isReply ? "rp" : "sd",
        targetCoreId, epid, dtu.regs().get(CmdReg::DATA_ADDR), messageSize);
    DPRINTF(Dtu, "  header: tgtEP=%u, lbl=%#018lx, rpLbl=%#018lx, rpEP=%u\n",
        targetEpId, label, replyLabel, replyEpId);

    if (targetCoreId == 0 && !isReply)
    {
        size_t sysNo = data[0];
        DPRINTF(DtuSysCalls, "  syscall: %s\n",
            sysNo < (sizeof(syscallNames) / sizeof(syscallNames[0])) ? syscallNames[sysNo] : "Unknown");
    }

    DPRINTF(DtuDetail, "Send %s of %lu bytes to EP%u at PE%u.\n",
                 isReply ? "reply" : "message",
                 messageSize,
                 targetEpId,
                 targetCoreId);

    assert(dtu.regs().get(CmdReg::DATA_SIZE) == messageSize);

    Dtu::MessageHeader header;

    if (isReply)
        header.flags = Dtu::REPLY_FLAG | Dtu::GRANT_CREDITS_FLAG;
    else
        header.flags = Dtu::REPLY_ENABLED; // normal message

    header.senderCoreId = static_cast<uint8_t>(dtu.coreId);
    header.senderEpId   = static_cast<uint8_t>(epid);
    header.replyEpId    = static_cast<uint8_t>(replyEpId);
    header.length       = static_cast<uint16_t>(messageSize);
    header.label        = static_cast<uint64_t>(label);
    header.replyLabel   = static_cast<uint64_t>(replyLabel);

    auto pkt = dtu.generateRequest(dtu.getNocAddr(targetCoreId, targetEpId),
                               messageSize + sizeof(Dtu::MessageHeader),
                               MemCmd::WriteReq);

    memcpy(pkt->getPtr<uint8_t>(), &header, sizeof(Dtu::MessageHeader));
    memcpy(pkt->getPtr<uint8_t>() + sizeof(Dtu::MessageHeader),
           data,
           messageSize);

    /*
     * The message data is derived from a scratchpad response. Therefore
     * we need to pay for the header delay of this reponse in addition
     * to the delay caused by the DTU. We don't need to pay for the
     * payload delay as data is forwarded word by word. Therefore the
     * payload delay is assigned to the new NoC packet, so that the overall
     * payload delay is defined by the slowest component (scratchpad or NoC)
     */
    Cycles delay = dtu.spmResponseToNocRequestLatency;
    delay += dtu.ticksToCycles(spmPktHeaderDelay);

    if (isReply)
        delay += Cycles(3); // pay for the functional request
                            // TODO check value

    pkt->payloadDelay = spmPktPayloadDelay;
    dtu.printPacket(pkt);
    dtu.sendNocRequest(pkt, delay, true);
}

void
MessageUnit::incrementReadPtr(unsigned epId)
{
    Addr readPtr    = dtu.regs().get(epId, EpReg::BUF_RD_PTR);
    Addr bufferAddr = dtu.regs().get(epId, EpReg::BUF_ADDR);
    Addr bufferSize = dtu.regs().get(epId, EpReg::BUF_SIZE);
    Addr messageCount = dtu.regs().get(epId, EpReg::BUF_MSG_CNT);
    unsigned maxMessageSize = dtu.regs().get(epId, EpReg::BUF_MSG_SIZE);

    readPtr += maxMessageSize;

    if (readPtr >= bufferAddr + bufferSize * maxMessageSize)
        readPtr = bufferAddr;

    DPRINTF(DtuBuf, "EP%u: increment read pointer to %#018lx (msgCount=%u)\n",
                 epId,
                 readPtr,
                 messageCount - 1);

    // TODO error handling
    assert(messageCount != 0);

    /*
     * XXX Actually an additianally cycle is needed to update the register.
     *     We ignore this delay as it should have no or a very small influence
     *     on the performance of the simulated system.
     */

    dtu.regs().set(epId, EpReg::BUF_RD_PTR, readPtr);
    dtu.regs().set(epId, EpReg::BUF_MSG_CNT, messageCount - 1);

    dtu.updateSuspendablePin();
}

bool
MessageUnit::incrementWritePtr(unsigned epId)
{
    Addr writePtr     = dtu.regs().get(epId, EpReg::BUF_WR_PTR);
    Addr bufferAddr   = dtu.regs().get(epId, EpReg::BUF_ADDR);
    Addr bufferSize   = dtu.regs().get(epId, EpReg::BUF_SIZE);
    Addr messageCount = dtu.regs().get(epId, EpReg::BUF_MSG_CNT);
    unsigned maxMessageSize = dtu.regs().get(epId, EpReg::BUF_MSG_SIZE);

    writePtr += maxMessageSize;

    if (writePtr >= bufferAddr + bufferSize * maxMessageSize)
        writePtr = bufferAddr;

    DPRINTF(DtuBuf, "EP%u: increment write pointer to %#018lx (msgCount=%u)\n",
                 epId,
                 writePtr,
                 messageCount + 1);

    if(messageCount == bufferSize)
    {
        warn("EP%u: Buffer full!\n", epId);
        return false;
    }

    dtu.regs().set(epId, EpReg::BUF_WR_PTR, writePtr);
    dtu.regs().set(epId, EpReg::BUF_MSG_CNT, messageCount + 1);

    dtu.updateSuspendablePin();
    dtu.wakeupCore();
    return true;
}

void
MessageUnit::recvFromNoc(PacketPtr pkt)
{
    assert(pkt->isWrite());
    assert(pkt->hasData());

    unsigned epId = pkt->getAddr() & ((1UL << dtu.nocEpAddrBits) - 1);

    Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

    Addr spmAddr = dtu.regs().get(epId, EpReg::BUF_WR_PTR);

    DPRINTF(Dtu, "\e[1m[rv <- %u]\e[0m %lu bytes on EP%u to %#018lx\n",
        header->senderCoreId, header->length, epId, spmAddr);
    dtu.printPacket(pkt);

    // Note that replyEpId is the Id of *our* sending EP
    if (header->flags & Dtu::REPLY_FLAG &&
        header->flags & Dtu::GRANT_CREDITS_FLAG &&
        header->replyEpId < dtu.numEndpoints)
    {
        unsigned maxMessageSize = dtu.regs().get(header->replyEpId, EpReg::MAX_MSG_SIZE);
        DPRINTF(DtuDetail, "Grant EP%u %u credits\n", header->replyEpId, maxMessageSize);

        unsigned credits = dtu.regs().get(header->replyEpId, EpReg::CREDITS);
        credits += maxMessageSize;
        dtu.regs().set(header->replyEpId, EpReg::CREDITS, credits);
    }

    if(incrementWritePtr(epId))
    {
        DPRINTF(DtuBuf, "EP%u: writing message to %#018lx\n", epId, spmAddr);

        pkt->setAddr(spmAddr);

        Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;
        delay += dtu.nocMessageToSpmRequestLatency;

        dtu.sendSpmRequest(pkt, epId, delay, Dtu::SpmPacketType::FORWARDED_MESSAGE);
    }
    else
    {
        pkt->makeResponse();

        if (!dtu.atomicMode)
        {
            Cycles delay = dtu.ticksToCycles(pkt->headerDelay + pkt->payloadDelay);
            delay += dtu.spmResponseToNocResponseLatency;

            pkt->headerDelay = 0;
            pkt->payloadDelay = 0;

            dtu.schedNocResponse(pkt, dtu.clockEdge(delay));
        }
    }
}

void
MessageUnit::recvFromNocComplete(PacketPtr pkt, unsigned epId)
{
    assert(pkt->isWrite());

    M5_VAR_USED Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

    if (!dtu.atomicMode)
    {
        DPRINTF(DtuDetail, "Wrote message to Scratchpad. Send response back to EP%u at PE%u\n",
                     header->senderEpId,
                     header->senderCoreId);

        Cycles delay = dtu.ticksToCycles(pkt->headerDelay + pkt->payloadDelay);
        delay += dtu.spmResponseToNocResponseLatency;

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        dtu.schedNocResponse(pkt, dtu.clockEdge(delay));
    }
}
