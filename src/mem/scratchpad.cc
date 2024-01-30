/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2021 Nils Asmussen, Barkhausen Institut
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

#include "mem/scratchpad.hh"
#include "debug/MemoryAccess.hh"

namespace gem5
{

namespace memory
{

Scratchpad::Scratchpad(const ScratchpadParams &p)
  : AbstractMemory(p),
    cpuPort(name() + ".cpu_port", *this),
    tcuPort(name() + ".tcu_port", *this),
    latency(p.latency),
    throughput(p.throughput),
    ignoreInvalid(p.ignore_invalid)
{
}

void
Scratchpad::init()
{
    AbstractMemory::init();

    assert(cpuPort.isConnected());

    cpuPort.sendRangeChange();
    if (tcuPort.isConnected())
        tcuPort.sendRangeChange();
}

Port &
Scratchpad::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_port") {
        return cpuPort;
    }
    else if (if_name == "tcu_port") {
        return tcuPort;
    }
    else {
        return SimObject::getPort(if_name, idx);
    }
}

bool
Scratchpad::checkInvalid(PacketPtr pkt)
{
    if (ignoreInvalid) {
        warn_if_once(pkt->getAddr() < getAddrRange().start(),
                     "%llu: %s: address out-of-range: %#x\n",
                     curTick(), name(), pkt->getAddr());

        // ignore invalid requests
        if (!pkt->getAddrRange().isSubset(getAddrRange()))
        {
            if (pkt->needsResponse())
                pkt->makeResponse();
            if (pkt->isRead())
                memset(pkt->getPtr<uint8_t>(), 0, pkt->getSize());
            return true;
        }
    }

    return false;
}

Tick
Scratchpad::recvAtomic(PacketPtr pkt)
{
    if (checkInvalid(pkt))
        return 0;

    fatal_if(!ignoreInvalid && !writeable && pkt->isWrite(),
             "%s: %s: write to read-only memory %s\n",
             curTick(), name(), pkt->getAddrRange().to_string());

    /*
     * TODO
     * So far the Scratchpad has no busy state -> it accepts all requests!
     */

    // let subclass handle the request
    access(pkt);

    unsigned cycles = round(((float)pkt->getSize()) / ((float)throughput));
    unsigned payloadDelay = clockPeriod() * cycles;

    // pay the header delay caused by interconnect and add the SPM latency
    unsigned totalDelay = pkt->headerDelay + latency * clockPeriod();
    pkt->headerDelay = 0;

    // Pay the payload dalay. The paylaod delay does not accumulate,
    // instead the highest value is taken.
    totalDelay += std::max(payloadDelay, pkt->payloadDelay);
    pkt->payloadDelay = 0;

    return totalDelay;
}

void
Scratchpad::recvFunctional(PacketPtr pkt)
{
    if (checkInvalid(pkt))
        return;

    functionalAccess(pkt);
}

Scratchpad::ScratchpadPort::ScratchpadPort(const std::string& _name,
                                           Scratchpad& _scratchpad)
    : SimpleTimingPort(_name, &_scratchpad), scratchpad(_scratchpad)
{ }

AddrRangeList
Scratchpad::ScratchpadPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(scratchpad.getAddrRange());
    return ranges;
}

Tick
Scratchpad::ScratchpadPort::recvAtomic(PacketPtr pkt)
{
    return scratchpad.recvAtomic(pkt);
}

void
Scratchpad::ScratchpadPort::recvFunctional(PacketPtr pkt)
{
    if (!respQueue.trySatisfyFunctional(pkt)) {
        // SimpleTimingPort calls recvAtomic() here, but we need to handle
        // functional accesses differently so that they bypass certain checks
        // such as writeable=False for ROMs. We still want to initialize those
        // with write requests during simulation startup.
        scratchpad.recvFunctional(pkt);
    }
}

}
}
