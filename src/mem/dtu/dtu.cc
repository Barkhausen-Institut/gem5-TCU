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
#include "sim/system.hh"

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    atomicMode(p->system->isAtomicMode()),
    regFile(name() + ".regFile", p->num_endpoints),
    numEndpoints(p->num_endpoints),
    masterId(p->system->getMasterId(name())),
    maxMessageSize(p->max_message_size),
    numCmdEpidBits(p->num_cmd_epid_bits),
    numCmdOffsetBits(p->num_cmd_offset_bits),
    registerAccessLatency(p->register_access_latency),
    commandToSpmRequestLatency(p->command_to_spm_request_latency),
    spmResponseToNocRequestLatency(p->spm_response_to_noc_request_latency),
    nocRequestToSpmRequestLatency(p->noc_request_to_spm_request_latency),
    spmResponseToNocResponseLatency(p->spm_response_to_noc_response_latency),
    executeCommandEvent(*this),
    finishOperationEvent(*this),
    incrementWritePtrEvent(*this)
{}

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

    cmd.epId = (reg & epidMask) >> numCmdEpidBits;

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
    case EpMode::RECEIVE_MESSAGE:
        // TODO Error handling
        panic("Ep %u: Cannot start operation on an endpoint"
              "that is configured to receive messages\n", cmd.epId);
        break;
    case EpMode::TRANSMIT_MESSAGE:
        startMessageTransmission(cmd);
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
    // reset command register and unset busy flag
    regFile.setDtuReg(DtuReg::COMMAND, 0);
    regFile.setDtuReg(DtuReg::STATUS, 0);
}

void
Dtu::sendSpmRequest(PacketPtr pkt, unsigned epId, Cycles delay, bool isForwarded)
{
    auto senderState = new SpmSenderState();
    senderState->epId = epId;
    senderState->isLocalRequest = !isForwarded;
    senderState->isForwardedRequest = isForwarded;

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
    Addr messageAddr = regFile.readEpReg(cmd.epId, EpReg::MESSAGE_ADDR);
    Addr messageSize = regFile.readEpReg(cmd.epId, EpReg::MESSAGE_SIZE);

    // TODO error handling
    assert(messageSize > 0);
    assert(messageSize + sizeof(MessageHeader) < maxMessageSize);

    DPRINTF(Dtu, "Endpoint %u starts message transmission.\n", cmd.epId);
    DPRINTF(Dtu, "Read message of %u Bytes at address %#x from local scratchpad.\n",
                 messageSize,
                 messageAddr);

    auto pkt = generateRequest(messageAddr, messageSize, MemCmd::ReadReq);

    sendSpmRequest(pkt, cmd.epId, commandToSpmRequestLatency, false);
}

void
Dtu::startMemoryWrite(const Command& cmd)
{
    Addr localAddr = regFile.readEpReg(cmd.epId, EpReg::REQUEST_LOCAL_ADDR);
    Addr requestSize = regFile.readEpReg(cmd.epId, EpReg::REQUEST_SIZE);

    // TODO error handling
    assert(requestSize > 0);
    assert(requestSize < maxMessageSize);

    Addr remoteAddr = regFile.readEpReg(cmd.epId, EpReg::REQUEST_SIZE);
    remoteAddr += cmd.offset;

    DPRINTF(Dtu, "Endpoint %u starts memory write.\n", cmd.epId);
    DPRINTF(Dtu, "Read %u bytes at address %#x from local scratchpad.\n",
                 requestSize,
                 localAddr);

    auto pkt = generateRequest(localAddr, requestSize, MemCmd::ReadReq);

    sendSpmRequest(pkt, cmd.epId, commandToSpmRequestLatency, false);
}

void
Dtu::incrementReadPtr(unsigned epId)
{
    Addr readPtr    = regFile.readEpReg(epId, EpReg::BUFFER_READ_PTR);
    Addr bufferAddr = regFile.readEpReg(epId, EpReg::BUFFER_ADDR);
    Addr bufferSize = regFile.readEpReg(epId, EpReg::BUFFER_SIZE);
    Addr messageCount = regFile.readEpReg(epId, EpReg::BUFFER_MESSAGE_COUNT);

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

    regFile.setEpReg(epId, EpReg::BUFFER_READ_PTR, readPtr);
    regFile.setEpReg(epId, EpReg::BUFFER_MESSAGE_COUNT, messageCount - 1);
}

void
Dtu::incrementWritePtr(unsigned epId)
{
    Addr writePtr     = regFile.readEpReg(epId, EpReg::BUFFER_WRITE_PTR);
    Addr bufferAddr   = regFile.readEpReg(epId, EpReg::BUFFER_ADDR);
    Addr bufferSize   = regFile.readEpReg(epId, EpReg::BUFFER_SIZE);
    Addr messageCount = regFile.readEpReg(epId, EpReg::BUFFER_MESSAGE_COUNT);

    assert(messageCount < bufferSize);

    writePtr += maxMessageSize;

    if (writePtr >= bufferAddr + bufferSize * maxMessageSize)
        writePtr = bufferAddr;

    DPRINTF(Dtu, "Ep %u: Increment the write pointer. New address: %#x\n",
                 epId,
                 writePtr);

    regFile.setEpReg(epId, EpReg::BUFFER_WRITE_PTR, writePtr);
    regFile.setEpReg(epId, EpReg::BUFFER_MESSAGE_COUNT, messageCount + 1);
}

void
Dtu::completeNocRequest(PacketPtr pkt)
{
    DPRINTF(Dtu, "Received response from remote DTU -> Transaction finished\n");

    Cycles delay = ticksToCycles(pkt->headerDelay + pkt->payloadDelay);

    // clean up
    delete pkt->req;
    delete pkt;

    schedule(finishOperationEvent, clockEdge(delay));
}

void
Dtu::completeSpmRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isResponse());

    DPRINTF(Dtu, "Received response from scratchpad.\n");

    auto senderState = dynamic_cast<SpmSenderState*>(pkt->popSenderState());

    assert(senderState->isLocalRequest || senderState->isForwardedRequest);
    assert(!(senderState->isLocalRequest && senderState->isForwardedRequest));

    if (senderState->isLocalRequest)
        completeLocalSpmRequest(pkt);
    else
        completeForwardedSpmRequest(pkt, senderState->epId);

    delete senderState;
}

void
Dtu::sendNocRequest(PacketPtr pkt, Cycles delay, bool isMessage)
{
    auto senderState = new NocSenderState();
    senderState->isMessage = isMessage;
    senderState->isMemoryRequest = !isMessage;

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
                    Tick spmPktHeaderDelay,
                    Tick spmPktPayloadDelay)
{
    unsigned epid = getCommand().epId;

    unsigned targetCoreId = regFile.readEpReg(epid, EpReg::TARGET_COREID);
    unsigned targetEpId   = regFile.readEpReg(epid, EpReg::TARGET_EPID);

    assert(regFile.readEpReg(epid, EpReg::MESSAGE_SIZE) == messageSize);
    assert(messageSize + sizeof(MessageHeader) <= maxMessageSize);

    DPRINTF(Dtu, "Send message of %u bytes to endpoint %u at core %u.\n",
                 messageSize,
                 targetEpId,
                 targetCoreId);

    MessageHeader header = { static_cast<uint8_t>(coreId),
                             static_cast<uint8_t>(epid),
                             static_cast<uint16_t>(messageSize) };

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

    Addr targetAddr = regFile.readEpReg(epId, EpReg::REQUEST_REMOTE_ADDR);
    targetAddr += cmd.offset;

    assert(requestSize == regFile.readEpReg(epId, EpReg::REQUEST_SIZE));
    assert(requestSize <= maxMessageSize);

    DPRINTF(Dtu, "Send %u bytes to address %#x.\n",
                 requestSize,
                 targetAddr);

    auto pkt = generateRequest(targetAddr,
                               requestSize,
                               MemCmd::WriteReq);
    memcpy(pkt->getPtr<uint8_t>() + sizeof(MessageHeader),
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

    if (mode == EpMode::TRANSMIT_MESSAGE)
        sendNocMessage(pkt->getConstPtr<uint8_t>(),
                       pkt->getSize(),
                       pkt->headerDelay,
                       pkt->payloadDelay);
    else if (mode == EpMode::WRITE_MEMORY)
        sendNocMemoryWriteRequest(pkt->getConstPtr<uint8_t>(),
                                  pkt->getSize(),
                                  pkt->headerDelay,
                                  pkt->payloadDelay);
    else
        // TODO error handling
        panic("Unexpected mode!\n");

    // clean up
    delete pkt->req;
    delete pkt;
}

void
Dtu::completeForwardedSpmRequest(PacketPtr pkt, unsigned epId)
{
    assert(pkt->isWrite());

    MessageHeader* header = pkt->getPtr<MessageHeader>();

    if (atomicMode)
    {
        incrementWritePtr(epId);
    }
    else
    {
        DPRINTF(Dtu, "Send response back to EP %u at core %u\n",
                     header->epId,
                     header->coreId);

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
Dtu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());

    auto senderState = dynamic_cast<NocSenderState*>(pkt->popSenderState());

    assert(senderState->isMessage || senderState->isMemoryRequest);
    assert(!(senderState->isMessage && senderState->isMemoryRequest));

    if (senderState->isMessage)
        recvNocMessage(pkt);
    else
        recvNocMemoryRequest(pkt);

    delete senderState;
}

void
Dtu::recvNocMessage(PacketPtr pkt)
{
    assert(pkt->isWrite());
    assert(pkt->hasData());

    unsigned epId = pkt->getAddr() & ((1UL << nocEpAddrBits) - 1);

    MessageHeader* header = pkt->getPtr<MessageHeader>();

    DPRINTF(Dtu, "EP %u received message of %u bytes from EP %u at core %u\n",
                 epId,
                 header->length,
                 header->epId,
                 header->coreId);

    unsigned messageCount = regFile.readEpReg(epId, EpReg::BUFFER_MESSAGE_COUNT);
    unsigned bufferSize   = regFile.readEpReg(epId, EpReg::BUFFER_SIZE);

    if (messageCount == bufferSize)
        // TODO error handling!
        panic("Ep %u: Buffer full!\n", epId);

    Addr spmAddr = regFile.readEpReg(epId, EpReg::BUFFER_WRITE_PTR);

    DPRINTF(Dtu, "Write message to local scratchpad at address %#x\n", spmAddr);

    pkt->setAddr(spmAddr);

    Cycles delay = ticksToCycles(pkt->headerDelay);
    delay += nocRequestToSpmRequestLatency;

    sendSpmRequest(pkt, epId, delay, true);
}

void
Dtu::recvNocMemoryRequest(PacketPtr pkt)
{
    // get local Address
    pkt->setAddr( pkt->getAddr() & ~getNocAddr(coreId, 0));

    // TODO Rename cpuBaseAddr to regFileBaseAddr
    if (pkt->getAddr() & cpuBaseAddr)
        forwardRequestToRegFile(pkt, false);
    else
        panic("Spm request cannot be handled yet\n");

    // restore global address for response
    pkt->setAddr(pkt->getAddr() | getNocAddr(coreId, 0));
}

void
Dtu::forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest)
{
    Addr origAddr = pkt->getAddr();

    // Strip the base address to handle requests based on the register address
    // only. The original address is restored before responding.
    pkt->setAddr(origAddr - cpuBaseAddr);

    bool commandWritten = regFile.handleRequest(pkt);

    pkt->setAddr(origAddr);

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
