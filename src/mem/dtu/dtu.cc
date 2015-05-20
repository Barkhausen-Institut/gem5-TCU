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
    registerAccessLatency(p->register_access_latency),
    commandToSpmRequestLatency(p->command_to_spm_request_latency),
    spmResponseToNocRequestLatency(p->spm_response_to_noc_request_latency),
    checkCommandAndExecuteEvent(*this),
    startTransactionEvent(*this)
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
Dtu::checkCommandAndExecute()
{
    RegFile::reg_t command = regFile.readDtuReg(DtuReg::COMMAND);

    // TODO paramterize commands?
    if (!(command & 0x100) || (command & 0xff) > numEndpoints)
        // TODO erro handling
        panic("Invalid command: %#x\n", command);

    startTransaction();
}

void
Dtu::startTransaction()
{
    // TODO paramterize commands?
    unsigned epid = regFile.readDtuReg(DtuReg::COMMAND) & 0xff;

    assert(epid < numEndpoints);

    if (regFile.readEpReg(epid, EpReg::CONFIG) != 1)
        panic("Issued transaction from EP %u but it is not configured for sending\n", epid);

    Addr messageAddr = regFile.readEpReg(epid, EpReg::MESSAGE_ADDR);
    Addr messageSize = regFile.readEpReg(epid, EpReg::MESSAGE_SIZE);

    // TODO error handling
    assert(messageSize > 0);
    assert(messageSize < maxMessageSize);

    DPRINTF(Dtu, "Start transmission.\n");
    DPRINTF(Dtu, "Read message of %u Bytes at address %#x from local scratchpad.\n",
                 messageSize,
                 messageAddr);

    // Reset command and set busy flag

    regFile.setDtuReg(DtuReg::COMMAND, 0);
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
Dtu::completeNocRequest(PacketPtr pkt)
{
    panic("Dtu::completeNocRequest() not yet implemented!\n");
}

void
Dtu::completeSpmRequest(PacketPtr pkt)
{
    panic("Dtu::completeSpmRequest() not yet implemented!\n");
}

void
Dtu::handleNocRequest(PacketPtr pkt)
{
    panic("Dtu::handleNocRequest() not yet implemented!\n");
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
            schedule(checkCommandAndExecuteEvent, when);
    }
    else if (commandWritten)
    {
        checkCommandAndExecute();
    }
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}
