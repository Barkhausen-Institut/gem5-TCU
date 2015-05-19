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

#include "mem/dtu/dtu.hh"
#include "sim/system.hh"

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    atomicMode(p->system->isAtomicMode()),
    regFile(name() + ".regFile", p->num_endpoints),
    registerAccessLatency(p->register_access_latency)
{}

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

    if (atomicMode)
    {
        if (regFile.handleRequest(pkt))
            ; // TODO check command reg and start a new transaction
    }
    else
    {
        Cycles transportDelay = ticksToCycles(pkt->headerDelay +
                                              pkt->payloadDelay);

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        /*
         * We handle the request immediatly although we should pay for
         * the transport delay and DTU latency. However, this is only needed
         * when the command register is written and therefore a new Transaction
         * needs to be started. Then the delay is payed by scheduling a future
         * event. If another register is accessed the delay is payed by
         * scheduling the response at some point in the future.
         */
        bool checkCommand = regFile.handleRequest(pkt);

        schedCpuResponse(pkt, clockEdge(transportDelay + registerAccessLatency));

        if (checkCommand)
            ; // TODO schedule a new event to check the command reg and start a
              //      new transaction
    }

    pkt->setAddr(origAddr);
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}
