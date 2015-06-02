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
    cmdEpidMask((1 << p->num_cmd_epid_bits) - 1),
    registerAccessLatency(p->register_access_latency),
    commandToSpmRequestLatency(p->command_to_spm_request_latency),
    spmResponseToNocRequestLatency(p->spm_response_to_noc_request_latency),
    nocRequestToSpmRequestLatency(p->noc_request_to_spm_request_latency),
    spmResponseToNocResponseLatency(p->spm_response_to_noc_response_latency),
    executeCommandEvent(*this),
    finishTransactionEvent(*this),
    incrementWritePtrEvent(*this)
{}

PacketPtr
Dtu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
{
    Request::Flags flags;

    auto req = new Request(paddr, size, flags, masterId);

    auto pkt = new Packet(req, cmd);
    auto pktData = new uint8_t[size];
    pkt->dataDynamic(pktData);

    return pkt;
}

void
Dtu::executeCommand()
{
    RegFile::reg_t cmdWord = regFile.readDtuReg(DtuReg::COMMAND);

    RegFile::reg_t epId = cmdWord & cmdEpidMask;
    Command cmd = static_cast<Command>(cmdWord >> numCmdEpidBits);

    assert(epId < numEndpoints);

    switch (cmd)
    {
    case Command::IDLE:
        break;
    case Command::SEND_MESSAGE:
        startTransaction(epId);
        break;
    case Command::INC_READ_PTR:
        incrementReadPtr(epId);
        break;
    default:
        // TODO error handling
        panic("Invalid command %#x\n", static_cast<RegFile::reg_t>(cmd));
    }
}

void
Dtu::startTransaction(unsigned epId)
{
    if (regFile.readEpReg(epId, EpReg::CONFIG) != 1)
        panic("Issued transaction from EP %u but it is not configured for sending\n", epId);

    Addr messageAddr = regFile.readEpReg(epId, EpReg::MESSAGE_ADDR);
    Addr messageSize = regFile.readEpReg(epId, EpReg::MESSAGE_SIZE);

    // TODO error handling
    assert(messageSize > 0);
    assert(messageSize + sizeof(MessageHeader) < maxMessageSize);

    DPRINTF(Dtu, "Endpoint %u starts transmission.\n", epId);
    DPRINTF(Dtu, "Read message of %u Bytes at address %#x from local scratchpad.\n",
                 messageSize,
                 messageAddr);

    // set busy flag
    regFile.setDtuReg(DtuReg::STATUS, 1);

    auto pkt = generateRequest(messageAddr, messageSize, MemCmd::ReadReq);

    if (atomicMode)
    {
        sendAtomicSpmRequest(pkt);
        completeSpmRequest(pkt);
    }
    else
        schedSpmRequest(pkt, clockEdge(commandToSpmRequestLatency));
}

void
Dtu::finishTransaction()
{
    // reset command register and unset busy flag
    regFile.setDtuReg(DtuReg::COMMAND, 0);
    regFile.setDtuReg(DtuReg::STATUS, 0);
}

void
Dtu::incrementReadPtr(unsigned epId)
{
    Addr readPtr    = regFile.readEpReg(epId, EpReg::BUFFER_READ_PTR);
    Addr bufferAddr = regFile.readEpReg(epId, EpReg::BUFFER_ADDR);
    Addr bufferSize = regFile.readEpReg(epId, EpReg::BUFFER_SIZE);

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
}

void
Dtu::incrementWritePtr(unsigned epId)
{
    /*
     * XXX We don't check for buffer overflow. We assume that the credit system
     *     woks correctly and avoids overflows.
     */

    Addr writePtr   = regFile.readEpReg(epId, EpReg::BUFFER_WRITE_PTR);
    Addr bufferAddr = regFile.readEpReg(epId, EpReg::BUFFER_ADDR);
    Addr bufferSize = regFile.readEpReg(epId, EpReg::BUFFER_SIZE);

    writePtr += maxMessageSize;

    if (writePtr >= bufferAddr + bufferSize * maxMessageSize)
        writePtr = bufferAddr;

    DPRINTF(Dtu, "Ep %u: Increment the write pointer. New address: %#x\n",
                 epId,
                 writePtr);

    regFile.setEpReg(epId, EpReg::BUFFER_WRITE_PTR, writePtr);
}

void
Dtu::completeNocRequest(PacketPtr pkt)
{
    DPRINTF(Dtu, "Received response from remote DTU -> Transaction finished\n");

    Cycles delay = ticksToCycles(pkt->headerDelay + pkt->payloadDelay);

    // clean up
    delete pkt->req;
    delete pkt;

    schedule(finishTransactionEvent, clockEdge(delay));
}

void
Dtu::completeSpmRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isResponse());

    DPRINTF(Dtu, "Received response from scratchpad.\n");

    if (pkt->isRead())
        completeSpmReadRequest(pkt);
    else if (pkt->isWrite())
        completeSpmWriteRequest(pkt);
    else
        panic("Unexpected packet type\n");
}

void
Dtu::completeSpmReadRequest(PacketPtr pkt)
{
    // TODO paramterize commands?
    unsigned epid = regFile.readDtuReg(DtuReg::COMMAND) & 0xff;

    unsigned targetCoreId = regFile.readEpReg(epid, EpReg::TARGET_COREID);
    unsigned targetEpId   = regFile.readEpReg(epid, EpReg::TARGET_EPID);
    unsigned messageSize  = regFile.readEpReg(epid, EpReg::MESSAGE_SIZE);

    assert(pkt->getSize() == messageSize);

    DPRINTF(Dtu, "Send message of %u bytes to endpoint %u at core %u.\n",
                 messageSize,
                 targetEpId,
                 targetCoreId);

    MessageHeader header = { static_cast<uint8_t>(coreId),
                             static_cast<uint8_t>(epid),
                             static_cast<uint16_t>(messageSize) };

    auto nocPkt = generateRequest(getNocAddr(targetCoreId, targetEpId),
                                  messageSize + sizeof(MessageHeader),
                                  MemCmd::WriteReq);

    memcpy(nocPkt->getPtr<uint8_t>(), &header, sizeof(MessageHeader));
    memcpy(nocPkt->getPtr<uint8_t>() + sizeof(MessageHeader),
           pkt->getPtr<uint8_t>(),
           messageSize);

    Tick pktHeaderDelay = pkt->headerDelay;
    // XXX is this the right way to go?
    nocPkt->payloadDelay = pkt->payloadDelay;

    // clean up
    delete pkt->req;
    delete pkt;

    if (atomicMode)
    {
        sendAtomicNocRequest(nocPkt);
        completeNocRequest(nocPkt);
    }
    else
    {
        Cycles delay = spmResponseToNocRequestLatency;
        delay += ticksToCycles(pktHeaderDelay);
        schedNocRequest(nocPkt, clockEdge(delay));
    }
}

void
Dtu::completeSpmWriteRequest(PacketPtr pkt)
{
    MessageHeader* header = pkt->getPtr<MessageHeader>();

    auto senderState = dynamic_cast<DtuSenderState*>(pkt->popSenderState());

    if (atomicMode)
    {
        incrementWritePtr(senderState->epId);
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

        incrementWritePtrEvent.epId = senderState->epId;
        schedule(incrementWritePtrEvent, clockEdge(delay));

        schedNocResponse(pkt, clockEdge(delay));
    }

    delete senderState;
}

void
Dtu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isWrite());
    assert(pkt->hasData());

    unsigned epId = pkt->getAddr() & ((1UL << nocEpAddrBits) - 1);

    MessageHeader* header = pkt->getPtr<MessageHeader>();

    DPRINTF(Dtu, "EP %u received message of %u bytes from EP %u at core %u\n",
                 epId,
                 header->length,
                 header->epId,
                 header->coreId);

    Addr spmAddr = regFile.readEpReg(epId, EpReg::BUFFER_WRITE_PTR);

    DPRINTF(Dtu, "Write message to local scratchpad at address %#x\n", spmAddr);

    pkt->setAddr(spmAddr);

    auto senderState = new DtuSenderState();
    senderState->epId = epId;

    pkt->pushSenderState(senderState);

    if (atomicMode)
    {
        sendAtomicSpmRequest(pkt);
        completeSpmRequest(pkt);
    }
    else
    {
        Cycles delay = ticksToCycles(pkt->headerDelay);
        delay += nocRequestToSpmRequestLatency;

        pkt->headerDelay = 0;

        schedSpmRequest(pkt, clockEdge(delay));
    }
}

void
Dtu::handleCpuRequest(PacketPtr pkt)
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

        schedCpuResponse(pkt, when);

        if (commandWritten)
            schedule(executeCommandEvent, when);
    }
    else if (commandWritten)
    {
        executeCommand();
    }
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}
