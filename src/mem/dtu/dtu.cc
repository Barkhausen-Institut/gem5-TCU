/*
 * Copyright (c) 2015, Christian Menard
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
#include "mem/dtu/dtu.hh"
#include "mem/page_table.hh"
#include "sim/system.hh"
#include "sim/process.hh"

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    atomicMode(p->system->isAtomicMode()),
    system(p->system),
    regFile(name() + ".regFile", p->num_endpoints),
    numEndpoints(p->num_endpoints),
    masterId(p->system->getMasterId(name())),
    maxNocPacketSize(p->max_noc_packet_size),
    numCmdEpidBits(p->num_cmd_epid_bits),
    numCmdOffsetBits(p->num_cmd_offset_bits),
    registerAccessLatency(p->register_access_latency),
    commandToSpmRequestLatency(p->command_to_spm_request_latency),
    commandToNocRequestLatency(p->command_to_noc_request_latency),
    spmResponseToNocRequestLatency(p->spm_response_to_noc_request_latency),
    nocMessageToSpmRequestLatency(p->noc_message_to_spm_request_latency),
    nocResponseToSpmRequestLatency(p->noc_message_to_spm_request_latency),
    nocRequestToSpmRequestLatency(p->noc_request_to_spm_request_latency),
    spmResponseToNocResponseLatency(p->spm_response_to_noc_response_latency),
    usePTable(p->use_ptable),
    executeCommandEvent(*this),
    finishOperationEvent(*this),
    incrementWritePtrEvent(*this)
{}

Addr
Dtu::translate(Addr vaddr)
{
    Addr paddr = vaddr;
    if (usePTable)
    {
        M5_VAR_USED auto pTable = system->threadContexts[coreId]->getProcessPtr()->pTable;
        assert(pTable != nullptr);
        assert(pTable->translate(vaddr, paddr));
    }

    return paddr;
}

PacketPtr
Dtu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
{
    assert(numCmdEpidBits + numCmdOffsetBits + numCmdOpcodeBits <=
            sizeof(RegFile::reg_t) * 8);

    Request::Flags flags;

    auto req = new Request(paddr, size, flags, masterId);

    auto pkt = new Packet(req, cmd);
    auto pktData = new uint8_t[size];
    pkt->dataDynamic(pktData);

    return pkt;
}

Dtu::Command
Dtu::getCommand()
{
    using reg_t = RegFile::reg_t;

    /*
     *   COMMAND                        0
     * |--------------------------------|
     * |  offset  |   epid   |  opcode  |
     * |--------------------------------|
     */
    reg_t opcodeMask = (1 << numCmdOpcodeBits) - 1;
    reg_t epidMask   = ((1 << numCmdEpidBits) - 1) << numCmdOpcodeBits;
    reg_t offsetMask = ((1 << numCmdOffsetBits) - 1) << (numCmdOpcodeBits + numCmdEpidBits);

    auto reg = regFile.readDtuReg(DtuReg::COMMAND);

    Command cmd;

    cmd.opcode = static_cast<CommandOpcode>(reg & opcodeMask);

    cmd.epId = (reg & epidMask) >> numCmdOpcodeBits;

    cmd.offset = (reg & offsetMask) >> (numCmdEpidBits + numCmdOpcodeBits);

    return cmd;
}

void
Dtu::executeCommand()
{
    Command cmd = getCommand();

    assert(cmd.epId < numEndpoints);

    switch (cmd.opcode)
    {
    case CommandOpcode::IDLE:
        break;
    case CommandOpcode::START_OPERATION:
        startOperation(cmd);
        break;
    case CommandOpcode::INC_READ_PTR:
        incrementReadPtr(cmd.epId);
        break;
    default:
        // TODO error handling
        panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }
}

void
Dtu::startOperation(Command& cmd)
{
    // set busy flag
    regFile.setDtuReg(DtuReg::STATUS, 1);

    EpMode mode = static_cast<EpMode>(regFile.readEpReg(cmd.epId, EpReg::MODE));

    switch (mode)
    {
    case EpMode::RECEIVE_MESSAGE: // do a reply
    case EpMode::TRANSMIT_MESSAGE:
        startMessageTransmission(cmd);
        break;
    case EpMode::READ_MEMORY:
        startMemoryRead(cmd);
        break;
    case EpMode::WRITE_MEMORY:
        startMemoryWrite(cmd);
        break;
    default:
        // TODO Error handling
        panic("Ep %u: Invalid mode\n", cmd.epId);
    }
}

void
Dtu::finishOperation()
{
    DPRINTF(Dtu, "Operation finished\n");
    // reset command register and unset busy flag
    regFile.setDtuReg(DtuReg::COMMAND, 0);
    regFile.setDtuReg(DtuReg::STATUS, 0);
}

void
Dtu::sendSpmRequest(PacketPtr pkt,
                    unsigned epId,
                    Cycles delay,
                    SpmPacketType packetType)
{
    pkt->setAddr(translate(pkt->getAddr()));

    auto senderState = new SpmSenderState();
    senderState->epId = epId;
    senderState->packetType = packetType;

    pkt->pushSenderState(senderState);

    if (atomicMode)
    {
        sendAtomicSpmRequest(pkt);
        completeSpmRequest(pkt);
    }
    else
    {
        schedSpmRequest(pkt, clockEdge(delay));
    }
}

void
Dtu::startMessageTransmission(const Command& cmd)
{
    Addr messageAddr = regFile.readEpReg(cmd.epId, EpReg::MSG_ADDR);
    Addr messageSize = regFile.readEpReg(cmd.epId, EpReg::MSG_SIZE);
    unsigned maxMessageSize = regFile.readEpReg(cmd.epId, EpReg::MAX_MSG_SIZE);
    unsigned credits = regFile.readEpReg(cmd.epId, EpReg::CREDITS);

    EpMode mode = static_cast<EpMode>(regFile.readEpReg(cmd.epId, EpReg::MODE));

    /*
     * If this endpoint is configured to send messages, we need to check
     * credits. If it is configured to receive messages we do a reply and don't
     * need to check credits.
     */
    if (mode == EpMode::TRANSMIT_MESSAGE)
    {
        if (credits < maxMessageSize)
        {
            warn("pe%u.ep%u: Ignore send message commad because there are not "
                 "enough credits", coreId, cmd.epId);
            schedule(finishOperationEvent, clockEdge(Cycles(1)));
            return;
        }

        DPRINTF(Dtu, "EP%u pays %u credits\n", cmd.epId, maxMessageSize);

        // Pay some credits
        credits -= maxMessageSize;
        regFile.setEpReg(cmd.epId, EpReg::CREDITS, credits);
    }

    // TODO error handling
    assert(messageSize > 0);
    assert(messageSize + sizeof(MessageHeader) <= maxMessageSize);
    assert(messageSize + sizeof(MessageHeader) <= maxNocPacketSize);

    DPRINTF(Dtu, "Endpoint %u starts message transmission.\n", cmd.epId);
    DPRINTF(Dtu, "Read message of %u Bytes at address %#x from local scratchpad.\n",
                 messageSize,
                 messageAddr);

    auto pkt = generateRequest(messageAddr, messageSize, MemCmd::ReadReq);

    sendSpmRequest(pkt,
                   cmd.epId,
                   commandToSpmRequestLatency,
                   SpmPacketType::LOCAL_REQUEST);
}

void
Dtu::startMemoryRead(const Command& cmd)
{
    Addr requestSize = regFile.readEpReg(cmd.epId, EpReg::REQ_SIZE);
    Addr remoteAddr = regFile.readEpReg(cmd.epId, EpReg::REQ_REM_ADDR);
    remoteAddr += cmd.offset;

    // TODO error handling
    assert(requestSize > 0);
    assert(requestSize < maxNocPacketSize);

    DPRINTF(Dtu, "Endpoint %u starts memory read.\n", cmd.epId);
    DPRINTF(Dtu, "Read %u bytes from global address %#x\n",
                 requestSize,
                 remoteAddr);

    auto pkt = generateRequest(remoteAddr, requestSize, MemCmd::ReadReq);

    sendNocRequest(pkt,
                   commandToNocRequestLatency,
                   false);
}

void
Dtu::startMemoryWrite(const Command& cmd)
{
    Addr localAddr = regFile.readEpReg(cmd.epId, EpReg::REQ_LOC_ADDR);
    Addr requestSize = regFile.readEpReg(cmd.epId, EpReg::REQ_SIZE);

    // TODO error handling
    assert(requestSize > 0);
    assert(requestSize <= maxNocPacketSize);

    Addr remoteAddr = regFile.readEpReg(cmd.epId, EpReg::REQ_SIZE);
    remoteAddr += cmd.offset;

    DPRINTF(Dtu, "Endpoint %u starts memory write.\n", cmd.epId);
    DPRINTF(Dtu, "Read %u bytes at address %#x from local scratchpad.\n",
                 requestSize,
                 localAddr);

    auto pkt = generateRequest(localAddr, requestSize, MemCmd::ReadReq);

    sendSpmRequest(pkt,
                   cmd.epId,
                   commandToSpmRequestLatency,
                   SpmPacketType::LOCAL_REQUEST);
}

void
Dtu::incrementReadPtr(unsigned epId)
{
    Addr readPtr    = regFile.readEpReg(epId, EpReg::BUF_RD_PTR);
    Addr bufferAddr = regFile.readEpReg(epId, EpReg::BUF_ADDR);
    Addr bufferSize = regFile.readEpReg(epId, EpReg::BUF_SIZE);
    Addr messageCount = regFile.readEpReg(epId, EpReg::BUF_MSG_CNT);
    unsigned maxMessageSize = regFile.readEpReg(epId, EpReg::MAX_MSG_SIZE);

    // TODO error handling
    assert(messageCount != 0);

    readPtr += maxMessageSize;

    if (readPtr >= bufferAddr + bufferSize * maxMessageSize)
        readPtr = bufferAddr;

    DPRINTF(Dtu, "Ep %u: Increment the read pointer. New address: %#x\n",
                 epId,
                 readPtr);

    /*
     * XXX Actually an additianally cycle is needed to update the register.
     *     We ignore this delay as it should have no or a very small influence
     *     on the performance of the simulated system.
     */

    regFile.setEpReg(epId, EpReg::BUF_RD_PTR, readPtr);
    regFile.setEpReg(epId, EpReg::BUF_MSG_CNT, messageCount - 1);
}

void
Dtu::incrementWritePtr(unsigned epId)
{
    Addr writePtr     = regFile.readEpReg(epId, EpReg::BUF_WR_PTR);
    Addr bufferAddr   = regFile.readEpReg(epId, EpReg::BUF_ADDR);
    Addr bufferSize   = regFile.readEpReg(epId, EpReg::BUF_SIZE);
    Addr messageCount = regFile.readEpReg(epId, EpReg::BUF_MSG_CNT);
    unsigned maxMessageSize = regFile.readEpReg(epId, EpReg::MAX_MSG_SIZE);

    assert(messageCount < bufferSize);

    writePtr += maxMessageSize;

    if (writePtr >= bufferAddr + bufferSize * maxMessageSize)
        writePtr = bufferAddr;

    DPRINTF(Dtu, "Ep %u: Increment the write pointer. New address: %#x\n",
                 epId,
                 writePtr);

    regFile.setEpReg(epId, EpReg::BUF_WR_PTR, writePtr);
    regFile.setEpReg(epId, EpReg::BUF_MSG_CNT, messageCount + 1);
}

void
Dtu::completeNocRequest(PacketPtr pkt)
{
    DPRINTF(Dtu, "Received %s response from remote DTU.\n",
                 pkt->isRead() ? "read" : "write");

    if (pkt->isWrite())
        completeNocWriteRequest(pkt);
    else if (pkt->isRead())
        completeNocReadRequest(pkt);
    else
        panic("unexpected packet type\n");

    // clean up
    delete pkt->req;
    delete pkt;
}

void
Dtu::completeNocWriteRequest(PacketPtr pkt)
{
    Cycles delay = ticksToCycles(pkt->headerDelay + pkt->payloadDelay);

    schedule(finishOperationEvent, clockEdge(delay));
}

void
Dtu::completeNocReadRequest(PacketPtr pkt)
{

    auto cmd = getCommand();

    Addr localAddr = regFile.readEpReg(cmd.epId, EpReg::REQ_LOC_ADDR);
    Addr requestSize = regFile.readEpReg(cmd.epId, EpReg::REQ_SIZE);

    DPRINTF(Dtu, "Write %u bytes to local scratchpad at address %#x.\n",
                 requestSize,
                 localAddr);

    Cycles delay = ticksToCycles(pkt->headerDelay);

    auto spmPkt = generateRequest(localAddr, requestSize, MemCmd::ReadReq);
    spmPkt->payloadDelay = pkt->payloadDelay;

    memcpy(spmPkt->getPtr<uint8_t>(), pkt->getPtr<uint8_t>(), requestSize);

    sendSpmRequest(spmPkt,
                   cmd.epId,
                   delay + nocResponseToSpmRequestLatency,
                   SpmPacketType::LOCAL_REQUEST);
}

void
Dtu::completeSpmRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isResponse());

    DPRINTF(Dtu, "Received response from scratchpad.\n");

    auto senderState = dynamic_cast<SpmSenderState*>(pkt->popSenderState());

    switch (senderState->packetType)
    {
    case SpmPacketType::LOCAL_REQUEST:
        completeLocalSpmRequest(pkt);
        break;
    case SpmPacketType::FORWARDED_MESSAGE:
        completeForwardedMessage(pkt, senderState->epId);
        break;
    case SpmPacketType::FORWARDED_REQUEST:
        completeForwardedRequest(pkt);
        break;
    default:
        panic("Unexpected SpmPacketType\n");
    }

    delete senderState;
}

void
Dtu::sendNocRequest(PacketPtr pkt, Cycles delay, bool isMessage)
{
    auto senderState = new NocSenderState();
    senderState->packetType = isMessage ? NocPacketType::MESSAGE :
                                          NocPacketType::REQUEST;

    pkt->pushSenderState(senderState);

    if (atomicMode)
    {
        sendAtomicNocRequest(pkt);
        completeNocRequest(pkt);
    }
    else
    {
        schedNocRequest(pkt, clockEdge(delay));
    }
}

void
Dtu::sendNocMessage(const uint8_t* data,
                    Addr messageSize,
                    bool isReply,
                    Tick spmPktHeaderDelay,
                    Tick spmPktPayloadDelay)
{
    unsigned epid = getCommand().epId;

    unsigned targetCoreId;
    unsigned targetEpId;
    unsigned replyEpId;
    uint64_t label;
    uint64_t replyLabel;

    M5_VAR_USED unsigned maxMessageSize = regFile.readEpReg(epid, EpReg::MAX_MSG_SIZE);

    if (isReply)
    {
        /*
         * We need to read the header of the received message from scratchpad
         * to determine target core and enspoint ID. This would introduce a
         * second scratchpad request and would make the control flow more
         * complicated. To simplify things a functional request is used and an
         * additional delay is payed.
         */

        auto pkt = generateRequest(
                translate(regFile.readEpReg(epid, EpReg::BUF_RD_PTR)),
                sizeof(MessageHeader),
                MemCmd::ReadReq);

        spmMasterPort.sendFunctional(pkt);

        auto h = pkt->getPtr<MessageHeader>();

        targetCoreId = h->senderCoreId;
        targetEpId   = h->replyEpId;  // send messgae to the reply EP
        replyEpId    = h->senderEpId; // and grand credits to the sender

        // the receiver of the reply should get the label that he has set
        label        = h->replyLabel;
        // replies don't have replies. so, we don't need that
        replyLabel   = 0;
    }
    else
    {
        targetCoreId = regFile.readEpReg(epid, EpReg::TGT_COREID);
        targetEpId   = regFile.readEpReg(epid, EpReg::TGT_EPID);
        replyEpId    = regFile.readEpReg(epid, EpReg::REPLY_EPID);
        label        = regFile.readEpReg(epid, EpReg::LABEL);
        replyLabel   = regFile.readEpReg(epid, EpReg::REPLY_LABEL);
    }

    assert(regFile.readEpReg(epid, EpReg::MSG_SIZE) == messageSize);
    assert(messageSize + sizeof(MessageHeader) <= maxMessageSize);

    DPRINTF(Dtu, "Send %s of %u bytes to endpoint %u at core %u.\n",
                 isReply ? "reply" : "message",
                 messageSize,
                 targetEpId,
                 targetCoreId);

    MessageHeader header;

    if (isReply)
        header.flags = REPLY_FLAG | GRAND_CREDITS_FLAG;
    else
        header.flags = 0; // normal message

    header.senderCoreId = static_cast<uint8_t>(coreId);
    header.senderEpId   = static_cast<uint8_t>(epid);
    header.replyEpId    = static_cast<uint8_t>(replyEpId);
    header.length       = static_cast<uint16_t>(messageSize);
    header.label        = static_cast<uint64_t>(label);
    header.replyLabel   = static_cast<uint64_t>(replyLabel);

    auto pkt = generateRequest(getNocAddr(targetCoreId, targetEpId),
                               messageSize + sizeof(MessageHeader),
                               MemCmd::WriteReq);

    memcpy(pkt->getPtr<uint8_t>(), &header, sizeof(MessageHeader));
    memcpy(pkt->getPtr<uint8_t>() + sizeof(MessageHeader),
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
    Cycles delay = spmResponseToNocRequestLatency;
    delay += ticksToCycles(spmPktHeaderDelay);

    if (isReply)
        delay += Cycles(2); // pay for the functional request
                            // TODO check value

    pkt->payloadDelay = spmPktPayloadDelay;
    sendNocRequest(pkt, delay, true);
}

void
Dtu::sendNocMemoryWriteRequest(const uint8_t* data,
                               Addr requestSize,
                               Tick spmPktHeaderDelay,
                               Tick spmPktPayloadDelay)
{
    Command cmd = getCommand();
    unsigned epId = cmd.epId;

    unsigned targetCoreId = regFile.readEpReg(epId, EpReg::TGT_COREID);
    Addr targetAddr = regFile.readEpReg(epId, EpReg::REQ_REM_ADDR);
    targetAddr += cmd.offset;

    assert(requestSize == regFile.readEpReg(epId, EpReg::REQ_SIZE));
    assert(requestSize <= maxNocPacketSize);

    DPRINTF(Dtu, "Send %u bytes to address %#x in PE%u.\n",
                 requestSize,
                 targetAddr,
                 targetCoreId);

    auto pkt = generateRequest(getNocAddr(targetCoreId) | targetAddr,
                               requestSize,
                               MemCmd::WriteReq);
    memcpy(pkt->getPtr<uint8_t>(),
           data,
           requestSize);

    /*
     * See sendNocMessage() for an explanation of delay handling.
     */
    Cycles delay = spmResponseToNocRequestLatency;
    delay += ticksToCycles(spmPktHeaderDelay);
    pkt->payloadDelay = spmPktPayloadDelay;
    sendNocRequest(pkt, delay, false);
}

void
Dtu::completeLocalSpmRequest(PacketPtr pkt)
{
    assert(pkt->isRead());

    unsigned epid = getCommand().epId;

    EpMode mode = static_cast<EpMode>(regFile.readEpReg(epid, EpReg::MODE));

    if (mode == EpMode::TRANSMIT_MESSAGE || mode == EpMode::RECEIVE_MESSAGE)
        sendNocMessage(pkt->getConstPtr<uint8_t>(),
                       pkt->getSize(),
                       mode == EpMode::RECEIVE_MESSAGE,
                       pkt->headerDelay,
                       pkt->payloadDelay);
    else if (mode == EpMode::WRITE_MEMORY)
        sendNocMemoryWriteRequest(pkt->getConstPtr<uint8_t>(),
                                  pkt->getSize(),
                                  pkt->headerDelay,
                                  pkt->payloadDelay);
    else if (mode == EpMode::READ_MEMORY)
    {
        Cycles delay = ticksToCycles(pkt->headerDelay + pkt->payloadDelay);
        schedule(finishOperationEvent, clockEdge(delay));
    }
    else
        // TODO error handling
        panic("Unexpected mode!\n");

    // clean up
    delete pkt->req;
    delete pkt;
}

void
Dtu::completeForwardedMessage(PacketPtr pkt, unsigned epId)
{
    assert(pkt->isWrite());

    M5_VAR_USED MessageHeader* header = pkt->getPtr<MessageHeader>();

    if (atomicMode)
    {
        incrementWritePtr(epId);
    }
    else
    {
        DPRINTF(Dtu, "Wrote message to Scratchpad. Send response back to EP %u at core %u\n",
                     header->senderEpId,
                     header->senderCoreId);

        Cycles delay = ticksToCycles(pkt->headerDelay + pkt->payloadDelay);
        delay += spmResponseToNocResponseLatency;

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        incrementWritePtrEvent.epId = epId;
        schedule(incrementWritePtrEvent, clockEdge(delay));

        schedNocResponse(pkt, clockEdge(delay));
    }
}

void
Dtu::completeForwardedRequest(PacketPtr pkt)
{
    if (!atomicMode)
    {
        DPRINTF(Dtu, "Forwarded request to Scratchpad. Send response back via NoC\n");

        Cycles delay = ticksToCycles(pkt->headerDelay + pkt->payloadDelay);
        delay += spmResponseToNocResponseLatency;

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        schedNocResponse(pkt, clockEdge(delay));
    }
}

void
Dtu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());

    auto senderState = dynamic_cast<NocSenderState*>(pkt->popSenderState());

    switch (senderState->packetType)
    {
    case NocPacketType::MESSAGE:
        recvNocMessage(pkt);
        break;
    case NocPacketType::REQUEST:
        recvNocMemoryRequest(pkt);
        break;
    default:
        panic("Unexpected NocPacketType\n");
    }

    delete senderState;
}

void
Dtu::recvNocMessage(PacketPtr pkt)
{
    assert(pkt->isWrite());
    assert(pkt->hasData());

    unsigned epId = pkt->getAddr() & ((1UL << nocEpAddrBits) - 1);

    MessageHeader* header = pkt->getPtr<MessageHeader>();

    DPRINTF(Dtu, "EP %u received %s of %u bytes from EP %u at core %u\n",
                 epId,
                 header->flags & REPLY_FLAG ? "reply" : "message",
                 header->length,
                 header->senderEpId,
                 header->senderCoreId);

    if (header->flags & REPLY_FLAG &&
        header->flags & GRAND_CREDITS_FLAG &&
        header->replyEpId < numEndpoints)
    {
        unsigned maxMessageSize = regFile.readEpReg(epId, EpReg::MAX_MSG_SIZE);
        DPRINTF(Dtu, "Grand EP%u %u credits\n", header->replyEpId, maxMessageSize);

        unsigned credits = regFile.readEpReg(header->replyEpId, EpReg::CREDITS);
        credits += maxMessageSize;
        regFile.setEpReg(header->replyEpId, EpReg::CREDITS, credits);
    }

    unsigned messageCount = regFile.readEpReg(epId, EpReg::BUF_MSG_CNT);
    unsigned bufferSize   = regFile.readEpReg(epId, EpReg::BUF_SIZE);

    if (messageCount == bufferSize)
        // TODO error handling!
        panic("Ep %u: Buffer full!\n", epId);

    Addr spmAddr = regFile.readEpReg(epId, EpReg::BUF_WR_PTR);

    DPRINTF(Dtu, "Write message to local scratchpad at address %#x\n", spmAddr);

    pkt->setAddr(spmAddr);

    Cycles delay = ticksToCycles(pkt->headerDelay);
    pkt->headerDelay = 0;
    delay += nocMessageToSpmRequestLatency;

    sendSpmRequest(pkt, epId, delay, SpmPacketType::FORWARDED_MESSAGE);
}

void
Dtu::recvNocMemoryRequest(PacketPtr pkt)
{
    // get local Address
    pkt->setAddr( pkt->getAddr() & ~getNocAddr(coreId, 0));

    DPRINTF(Dtu, "Received %s request of %u bytes at %#x\n",
                 pkt->isWrite() ? "write" : "read",
                 pkt->getSize(),
                 pkt->getAddr());

    if (pkt->getAddr() & regFileBaseAddr)
    {
        forwardRequestToRegFile(pkt, false);
    }
    else
    {
        Cycles delay = ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;
        delay += nocRequestToSpmRequestLatency;

        sendSpmRequest(pkt, -1, delay, SpmPacketType::FORWARDED_REQUEST);
    }
}

void
Dtu::forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest)
{
    // Strip the base address to handle requests based on the register address
    // only.
    pkt->setAddr(pkt->getAddr() - regFileBaseAddr);

    bool commandWritten = regFile.handleRequest(pkt);

    if (!atomicMode)
    {
        /*
         * We handle the request immediatly and do not care about timing. The
         * delay is payed by scheduling the response at some point in the
         * future. Additionaly a write operation on the command register needs
         * to schedule an event that executes this command at a future tick.
         */

        Cycles transportDelay =
            ticksToCycles(pkt->headerDelay + pkt->payloadDelay);

        Tick when = clockEdge(transportDelay + registerAccessLatency);

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        if (isCpuRequest)
            schedCpuResponse(pkt, when);
        else
            schedNocResponse(pkt, when);

        if (commandWritten)
            schedule(executeCommandEvent, when);
    }
    else if (commandWritten)
    {
        executeCommand();
    }
}

void
Dtu::handleCpuRequest(PacketPtr pkt)
{
    forwardRequestToRegFile(pkt, true);
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}
