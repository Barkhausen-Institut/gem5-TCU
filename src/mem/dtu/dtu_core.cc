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

#include "base/trace.hh"
#include "debug/Dtu.hh"
#include "mem/dtu/dtu_core.hh"
#include "sim/system.hh"

DtuCore::DtuCore(const DtuParams* p,
                 MasterPort& _spmPort,
                 MasterPort& _masterPort)
    : spmPort(_spmPort),
      masterPort(_masterPort),
      atomic(p->system->isAtomicMode())
{}

Tick
DtuCore::handleCpuRequest(PacketPtr pkt)
{
    // for now simply forward all requests to the scratchpad

    DPRINTF(Dtu, "Received %s at address 0x%x\n",
            pkt->isWrite() ? "write" : "read",
            pkt->getAddr());

    Addr addr = pkt->getAddr();
    addr &= 0x0fffffff;
    pkt->setAddr(addr);

    DPRINTF(Dtu, "Forward to scratchpad at address 0x%x\n",
            pkt->getAddr());

    sendSpmPkt(pkt);

    // TODO
    Tick dtuDelay = 0;

    Tick totalDelay = pkt->headerDelay + pkt->payloadDelay + dtuDelay;

    /*
     * The SimpleTimingPort already pays for the delay returned by recvAtomic
     *  -> reset the packet delay
     *
     * XXX I'm not sure if this is the right way to go. However, it seems
     *     better than simply ignoring the packet's delays as it is done for
     *     instance in SimpleMemory.
     */
    pkt->headerDelay  = 0;
    pkt->payloadDelay = 0;

    return totalDelay;
}

bool
DtuCore::sendSpmPkt(PacketPtr pkt)
{
    if (atomic)
    {
        spmPort.sendAtomic(pkt);
        // TODO complete the request
    }
    else
    {
        panic("Timing requests are not implemented");
    }

    return true;
}
