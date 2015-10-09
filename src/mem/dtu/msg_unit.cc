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

    Addr messageAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr messageSize = dtu.regs().get(CmdReg::DATA_SIZE);
    unsigned credits = dtu.regs().get(epid, EpReg::CREDITS);

    bool isReply = cmd.opcode == Dtu::CommandOpcode::REPLY;

    /*
     * If this endpoint is configured to send messages, we need to check
     * credits. If it is configured to receive messages we do a reply and don't
     * need to check credits.
     */
    if (!isReply)
    {
        unsigned maxMessageSize = dtu.regs().get(epid, EpReg::MAX_MSG_SIZE);

        // TODO error handling
        // TODO atm, we can send (nearly) arbitrary large replies
        assert(messageSize + sizeof(Dtu::MessageHeader) <= maxMessageSize);

        if (credits < maxMessageSize)
        {
            warn("pe%u.ep%u: Ignore send message command because there are not "
                 "enough credits", dtu.coreId, epid);
            dtu.scheduleFinishOp(Cycles(1));
            return;
        }

        DPRINTFS(DtuDetail, (&dtu), "EP%u pays %u credits\n", epid, maxMessageSize);

        // Pay some credits
        credits -= maxMessageSize;
        dtu.regs().set(epid, EpReg::CREDITS, credits);
    }

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

    DPRINTFS(Dtu, (&dtu), "\e[1m[%s -> %u]\e[0m with EP%u of %#018lx:%lu\n",
        isReply ? "rp" : "sd",
        targetCoreId, epid, dtu.regs().get(CmdReg::DATA_ADDR), messageSize);
    DPRINTFS(Dtu, (&dtu), "  header: tgtEP=%u, lbl=%#018lx, rpLbl=%#018lx, rpEP=%u\n",
        targetEpId, label, replyLabel, replyEpId);

    Dtu::MessageHeader* header = new Dtu::MessageHeader;

    if (isReply)
        header->flags = Dtu::REPLY_FLAG | Dtu::GRANT_CREDITS_FLAG;
    else
        header->flags = Dtu::REPLY_ENABLED; // normal message

    header->senderCoreId = static_cast<uint8_t>(dtu.coreId);
    header->senderEpId   = static_cast<uint8_t>(epid);
    header->replyEpId    = static_cast<uint8_t>(replyEpId);
    header->length       = static_cast<uint16_t>(messageSize);
    header->label        = static_cast<uint64_t>(label);
    header->replyLabel   = static_cast<uint64_t>(replyLabel);

    assert(messageSize + sizeof(Dtu::MessageHeader) <= dtu.maxNocPacketSize);

    // start the transfer of the payload
    dtu.startTransfer(Dtu::TransferType::LOCAL_READ,
                      NocAddr(targetCoreId, targetEpId),
                      messageAddr,
                      messageSize,
                      NULL,
                      header,
                      Cycles(3));    // pay for the functional request; TODO
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

    DPRINTFS(DtuBuf, (&dtu), "EP%u: increment read pointer to %#018lx (msgCount=%u)\n",
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

    DPRINTFS(DtuBuf, (&dtu), "EP%u: increment write pointer to %#018lx (msgCount=%u)\n",
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

    NocAddr addr(pkt->getAddr());

    unsigned epId = addr.epId;

    Addr spmAddr = dtu.regs().get(epId, EpReg::BUF_WR_PTR);
    
    Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

    DPRINTFS(Dtu, (&dtu), "\e[1m[rv <- %u]\e[0m %lu bytes on EP%u to %#018lx\n",
        header->senderCoreId, header->length, epId, spmAddr);
    dtu.printPacket(pkt);

    if(dtu.coreId == 0 && epId == 0)
    {
        size_t sysNo = pkt->getPtr<uint8_t>()[0];
        DPRINTFS(DtuSysCalls, (&dtu), "  syscall: %s\n",
            sysNo < (sizeof(syscallNames) / sizeof(syscallNames[0])) ? syscallNames[sysNo] : "Unknown");
    }

    Addr bufferSize = dtu.regs().get(epId, EpReg::BUF_SIZE);
    Addr messageCount = dtu.regs().get(epId, EpReg::BUF_MSG_CNT);

    if(messageCount < bufferSize)
    {
        Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

        // Note that replyEpId is the Id of *our* sending EP
        if (header->flags & Dtu::REPLY_FLAG &&
            header->flags & Dtu::GRANT_CREDITS_FLAG &&
            header->replyEpId < dtu.numEndpoints)
        {
            unsigned maxMessageSize = dtu.regs().get(header->replyEpId, EpReg::MAX_MSG_SIZE);
            DPRINTFS(DtuDetail, (&dtu), "Grant EP%u %u credits\n", header->replyEpId, maxMessageSize);

            unsigned credits = dtu.regs().get(header->replyEpId, EpReg::CREDITS);
            credits += maxMessageSize;
            dtu.regs().set(header->replyEpId, EpReg::CREDITS, credits);
        }

        Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;
        delay += dtu.nocMessageToSpmRequestLatency;

        dtu.startTransfer(Dtu::TransferType::REMOTE_WRITE,
                          NocAddr(0, 0),
                          spmAddr,
                          pkt->getSize(),
                          pkt,
                          NULL,
                          delay);

        incrementWritePtr(epId);
    }
    // ignore messages if there is not enough space
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
