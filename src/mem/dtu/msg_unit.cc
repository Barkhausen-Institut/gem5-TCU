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
    if(cmd.opcode == Dtu::CommandOpcode::REPLY)
    {
        offset = 0;
        requestHeader(cmd.epId);
        return;
    }

    // check if we have enough credits
    Addr messageSize = dtu.regs().get(CmdReg::DATA_SIZE);
    Addr maxMessageSize = dtu.regs().get(epid, EpReg::MAX_MSG_SIZE);
    unsigned credits = dtu.regs().get(epid, EpReg::CREDITS);

    // TODO error handling
    assert(messageSize + sizeof(Dtu::MessageHeader) <= maxMessageSize);

    if (credits < maxMessageSize)
    {
        warn("pe%u.ep%u: Ignore send message command because there are not "
             "enough credits", dtu.coreId, epid);
        dtu.scheduleFinishOp(Cycles(1));
        return;
    }

    credits -= maxMessageSize;

    DPRINTFS(DtuCredits, (&dtu), "EP%u pays %u credits (%u left)\n",
             epid, maxMessageSize, credits);

    // pay the credits
    dtu.regs().set(epid, EpReg::CREDITS, credits);

    // fill the info struct and start the transfer
    info.targetCoreId = dtu.regs().get(epid, EpReg::TGT_COREID);
    info.targetEpId   = dtu.regs().get(epid, EpReg::TGT_EPID);
    info.label        = dtu.regs().get(epid, EpReg::LABEL);
    info.replyLabel   = dtu.regs().get(CmdReg::REPLY_LABEL);
    info.replyEpId    = dtu.regs().get(CmdReg::REPLY_EPID);
    info.ready = true;

    startXfer(cmd);
}

void
MessageUnit::requestHeader(unsigned epid)
{
    assert(offset < sizeof(Dtu::MessageHeader));

    Addr msgAddr = dtu.regs().get(epid, EpReg::BUF_RD_PTR);

    DPRINTFS(DtuBuf, (&dtu), "EP%d: requesting header for reply on message @ %p\n",
             epid, msgAddr + offset);

    // take care that we might need 2 loads to request the header
    Addr blockOff = (msgAddr + offset) & (dtu.blockSize - 1);
    Addr reqSize = std::min(dtu.blockSize - blockOff, sizeof(Dtu::MessageHeader) - offset);

    auto pkt = dtu.generateRequest(msgAddr + offset,
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
    memcpy(reinterpret_cast<char*>(&header) + offset, pkt->getPtr<char*>(), pkt->getSize());

    offset += pkt->getSize();

    // do we have the complete header yet? if not, request the rest
    if(offset < sizeof(Dtu::MessageHeader))
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
    info.ready = true;

    // disable replies for this message
    // use a functional request here because we don't need to wait for it anyway
    Addr msgAddr = dtu.regs().get(cmd.epId, EpReg::BUF_RD_PTR);
    auto hpkt = dtu.generateRequest(msgAddr,
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
             info.targetCoreId, cmd.epId, dtu.regs().get(CmdReg::DATA_ADDR), messageSize);

    DPRINTFS(Dtu, (&dtu), "  header: tgtEP=%u, lbl=%#018lx, rpLbl=%#018lx, rpEP=%u\n",
             info.targetEpId, info.label, info.replyLabel, info.replyEpId);

    Dtu::MessageHeader* header = new Dtu::MessageHeader;

    if (cmd.opcode == Dtu::CommandOpcode::REPLY)
        header->flags = Dtu::REPLY_FLAG | Dtu::GRANT_CREDITS_FLAG;
    else
        header->flags = Dtu::REPLY_ENABLED; // normal message

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

    Addr localAddr = dtu.regs().get(epId, EpReg::BUF_WR_PTR);
    
    Dtu::MessageHeader* header = pkt->getPtr<Dtu::MessageHeader>();

    DPRINTFS(Dtu, (&dtu), "\e[1m[rv <- %u]\e[0m %lu bytes on EP%u to %#018lx\n",
        header->senderCoreId, header->length, epId, localAddr);
    dtu.printPacket(pkt);

    if(dtu.coreId == 0 && epId == 0)
    {
        size_t sysNo = pkt->getPtr<uint8_t>()[sizeof(*header) + 0];
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
            unsigned credits = dtu.regs().get(header->replyEpId, EpReg::CREDITS);
            credits += maxMessageSize;

            DPRINTFS(DtuCredits, (&dtu), "EP%u: received %u credits (%u in total)\n",
                     header->replyEpId, maxMessageSize, credits);

            dtu.regs().set(header->replyEpId, EpReg::CREDITS, credits);
        }

        // the message is transferred piece by piece; we can start as soon as we have the header
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
        pkt->makeResponse();

        if (!dtu.atomicMode)
        {
            Cycles delay = dtu.ticksToCycles(pkt->headerDelay + pkt->payloadDelay);
            delay += dtu.nocToTransferLatency;

            pkt->headerDelay = 0;
            pkt->payloadDelay = 0;

            dtu.schedNocRequestFinished(dtu.clockEdge(Cycles(1)));
            dtu.schedNocResponse(pkt, dtu.clockEdge(delay));
        }
    }
}
