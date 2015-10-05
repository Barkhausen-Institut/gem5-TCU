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

#include <iomanip>
#include <sstream>

#include "debug/Dtu.hh"
#include "debug/DtuBuf.hh"
#include "debug/DtuDetail.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuSysCalls.hh"
#include "debug/DtuPower.hh"
#include "cpu/simple/base.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/msg_unit.hh"
#include "mem/dtu/mem_unit.hh"
#include "mem/dtu/xfer_unit.hh"
#include "mem/page_table.hh"
#include "sim/system.hh"
#include "sim/process.hh"

/**
 * The general idea is to have burst transfers that are only used by messages. Bursts mean that a
 * path through the NoC is reserved for that transfer hop by hop by the first packet. Afterwards,
 * only packets of that burst transfer or regular transfers can go through that path. Regular
 * transfers are used for memory requests, because they are always 1 cacheline large and multiple
 * requests can be treated independently. For messages however, we have to garuantee that each
 * message arrives contiguously, i.e. messages do not interleave when two are received at one EP
 * simultaneously. This is garuanteed by bursts.
 * In order to prevent deadlocks in the NoC, memory transfers can't be done in bursts. Since message
 * transfers are only initiated by SW, a bursts can never initiate another burst, but only memory
 * transfers. Thus, we will always make progress.
 *
 * This variable indicates whether a burst transfer is running somewhere in the NoC. This does only
 * roughly simulate bursts of real NoCs, but simulates the worst case. Note that it is not enough
 * to block sends to a certain receiver, because sends might interfere on their ways through the
 * NoC. Thus, we assume that they always do by blocking the whole NoC.
 */
bool Dtu::nocBurstActive = false;

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    masterId(p->system->getMasterId(name())),
    usePTable(p->use_ptable),
    system(p->system),
    regFile(name() + ".regFile", p->num_endpoints),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, sizeof(MessageHeader))),
    executeCommandEvent(*this),
    finishCommandEvent(*this),
    atomicMode(p->system->isAtomicMode()),
    numEndpoints(p->num_endpoints),
    maxNocPacketSize(p->max_noc_packet_size),
    numCmdEpidBits(p->num_cmd_epid_bits),
    registerAccessLatency(p->register_access_latency),
    commandToSpmRequestLatency(p->command_to_spm_request_latency),
    commandToNocRequestLatency(p->command_to_noc_request_latency),
    spmResponseToNocRequestLatency(p->spm_response_to_noc_request_latency),
    nocMessageToSpmRequestLatency(p->noc_message_to_spm_request_latency),
    nocResponseToSpmRequestLatency(p->noc_message_to_spm_request_latency),
    nocRequestToSpmRequestLatency(p->noc_request_to_spm_request_latency),
    spmResponseToNocResponseLatency(p->spm_response_to_noc_response_latency),
    transferToSpmRequestLatency(0)
{}

Dtu::~Dtu()
{
    delete xferUnit;
    delete memUnit;
    delete msgUnit;
}

Addr
Dtu::translate(Addr vaddr)
{
    Addr paddr = vaddr;
    if (usePTable)
    {
        M5_VAR_USED auto pTable = system->threadContexts[0]->getProcessPtr()->pTable;
        assert(pTable != nullptr);
        assert(pTable->translate(vaddr, paddr));
    }

    return paddr;
}

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
Dtu::freeRequest(PacketPtr pkt)
{
    delete pkt->req;
    delete pkt;
}

Dtu::Command
Dtu::getCommand()
{
    assert(numCmdEpidBits + numCmdOpcodeBits <= sizeof(RegFile::reg_t) * 8);

    using reg_t = RegFile::reg_t;

    /*
     *   COMMAND            0
     * |--------------------|
     * |  epid   |  opcode  |
     * |--------------------|
     */
    reg_t opcodeMask = ((reg_t)1 << numCmdOpcodeBits) - 1;
    reg_t epidMask   = (((reg_t)1 << numCmdEpidBits) - 1) << numCmdOpcodeBits;

    auto reg = regFile.get(CmdReg::COMMAND);

    Command cmd;

    cmd.opcode = static_cast<CommandOpcode>(reg & opcodeMask);

    cmd.epId = (reg & epidMask) >> numCmdOpcodeBits;

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
    case CommandOpcode::SEND:
    case CommandOpcode::REPLY:
        // if there is already someone sending, try again later
        if(nocBurstActive)
        {
            schedule(executeCommandEvent, clockEdge(Cycles(1)));
            return;
        }

        msgUnit->startTransmission(cmd);
        break;
    case CommandOpcode::READ:
        memUnit->startRead(cmd);
        break;
    case CommandOpcode::WRITE:
        memUnit->startWrite(cmd);
        break;
    case CommandOpcode::INC_READ_PTR:
        msgUnit->incrementReadPtr(cmd.epId);
        finishCommand();
        break;
    case CommandOpcode::WAKEUP_CORE:
        wakeupCore();
        finishCommand();
        break;
    default:
        // TODO error handling
        panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }
}

void
Dtu::finishCommand()
{
    DPRINTF(DtuDetail, "Command finished\n");
    // reset command register
    regFile.set(CmdReg::COMMAND, 0);
}

void
Dtu::wakeupCore()
{
    if(system->threadContexts[0]->status() == ThreadContext::Suspended)
    {
        DPRINTF(DtuPower, "Waking up core\n");
        system->threadContexts[0]->activate();
    }
}

void
Dtu::updateSuspendablePin()
{
    bool pendingMsgs = regFile.get(DtuReg::MSG_CNT) > 0;
    bool hadPending = system->threadContexts[0]->getCpuPtr()->_denySuspend;
    system->threadContexts[0]->getCpuPtr()->_denySuspend = pendingMsgs;
    if(hadPending && !pendingMsgs)
        DPRINTF(DtuPower, "Core can be suspended\n");
}

void
Dtu::sendSpmRequest(PacketPtr pkt,
                    unsigned epId,
                    Cycles delay,
                    SpmPacketType packetType,
                    bool last)
{
    pkt->setAddr(translate(pkt->getAddr()));

    auto senderState = new SpmSenderState();
    senderState->epId = epId;
    senderState->packetType = packetType;
    senderState->mid = pkt->req->masterId();
    senderState->last = last;

    // ensure that this packet has our master id (not the id of a master in a different PE)
    pkt->req->setMasterId(masterId);

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
Dtu::sendNocRequest(NocPacketType type, PacketPtr pkt, Cycles delay)
{
    auto senderState = new NocSenderState();
    senderState->packetType = type;

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
Dtu::startTransfer(NocPacketType type,
                   NocAddr targetAddr,
                   Addr sourceAddr,
                   Addr size)
{
    xferUnit->startTransfer(type,
                            targetAddr,
                            sourceAddr,
                            size);
}

void
Dtu::transferData(NocPacketType type,
                  NocAddr targetAddr,
                  const void* data,
                  Addr size,
                  Tick spmPktHeaderDelay,
                  Tick spmPktPayloadDelay)
{
    xferUnit->sendToNoc(type,
                        targetAddr,
                        data,
                        size,
                        spmPktHeaderDelay,
                        spmPktPayloadDelay);
}

void
Dtu::completeNocRequest(PacketPtr pkt)
{
    DPRINTF(DtuDetail, "Received %s response from remote DTU.\n",
                 pkt->isRead() ? "read" : "write");

    if (pkt->isWrite())
        memUnit->writeComplete(pkt);
    else if (pkt->isRead())
        memUnit->readComplete(pkt);
    else
        panic("unexpected packet type\n");

    freeRequest(pkt);
}

void
Dtu::completeSpmRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isResponse());

    DPRINTF(DtuDetail, "Received response from scratchpad.\n");

    auto senderState = dynamic_cast<SpmSenderState*>(pkt->popSenderState());

    // set the old master id again
    pkt->req->setMasterId(senderState->mid);

    switch (senderState->packetType)
    {
    case SpmPacketType::LOCAL_REQUEST:
        completeLocalSpmRequest(pkt, senderState->last);
        break;
    case SpmPacketType::FORWARDED_MESSAGE:
        msgUnit->recvFromNocComplete(pkt, senderState->epId);
        break;
    case SpmPacketType::FORWARDED_REQUEST:
        memUnit->recvFromNocComplete(pkt);
        break;
    case SpmPacketType::TRANSFER_REQUEST:
        xferUnit->forwardToNoc(pkt->getConstPtr<uint8_t>(),
                               pkt->headerDelay,
                               pkt->payloadDelay);
        break;
    default:
        panic("Unexpected SpmPacketType\n");
    }

    delete senderState;
}

void
Dtu::completeLocalSpmRequest(PacketPtr pkt, bool last)
{
    Command cmd = getCommand();

    if (cmd.opcode == CommandOpcode::WRITE)
    {
        memUnit->sendWriteToNoc(pkt->getConstPtr<uint8_t>(),
                                           pkt->getSize(),
                                           pkt->headerDelay,
                                           pkt->payloadDelay);
    }
    else if (cmd.opcode == CommandOpcode::READ)
    {
        memUnit->sendToSpmComplete(pkt, last);
    }
    else
        // TODO error handling
        panic("Unexpected mode!\n");

    freeRequest(pkt);
}

void
Dtu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());

    auto senderState = dynamic_cast<NocSenderState*>(pkt->popSenderState());

    switch (senderState->packetType)
    {
    case NocPacketType::MESSAGE:
        // if that failed, reply directly
        msgUnit->recvFromNoc(pkt);
        break;
    case NocPacketType::READ_REQ:
    case NocPacketType::WRITE_REQ:
        memUnit->recvFromNoc(pkt);
        break;
    default:
        panic("Unexpected NocPacketType\n");
    }

    delete senderState;
}

void
Dtu::handleCpuRequest(PacketPtr pkt)
{
    forwardRequestToRegFile(pkt, true);
}

void
Dtu::forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest)
{
    Addr oldAddr = pkt->getAddr();

    // Strip the base address to handle requests based on the register address only.
    pkt->setAddr(oldAddr - regFileBaseAddr);

    bool commandWritten = regFile.handleRequest(pkt, isCpuRequest);

    // restore old address
    pkt->setAddr(oldAddr);

    updateSuspendablePin();

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

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}

void
Dtu::printPacket(PacketPtr pkt) const
{
    DDUMP(DtuPackets, pkt->getPtr<uint8_t>(), pkt->getSize());
}
