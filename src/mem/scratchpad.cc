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

#include "mem/scratchpad.hh"

Scratchpad::Scratchpad(const ScratchpadParams* p)
  : AbstractMemory(p),
    cpuPort(name() + ".cpu_port", *this),
    dtuPort(name() + ".dtu_port", *this),
    latency(p->latency),
    throughput(p->throughput)
{
}

void
Scratchpad::init()
{
    AbstractMemory::init();

    assert(cpuPort.isConnected());

    cpuPort.sendRangeChange();
    if (dtuPort.isConnected())
        dtuPort.sendRangeChange();
}

BaseSlavePort &
Scratchpad::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_port") {
        return cpuPort;
    }
    else if (if_name == "dtu_port") {
        return dtuPort;
    }
    else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

Tick
Scratchpad::recvAtomic(PacketPtr pkt)
{
    // ignore invalid requests
    AddrRange pktRange(pkt->getAddr(), pkt->getAddr() + pkt->getSize() - 1);
    if (!pktRange.isSubset(getAddrRange()))
    {
        if (pkt->needsResponse())
            pkt->makeResponse();
        if (pkt->isRead())
            memset(pkt->getPtr<uint8_t>(), 0, pkt->getSize());
        return 0;
    }

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

Scratchpad::ScratchpadPort::ScratchpadPort(const std::string& _name,
                                           Scratchpad& _scratchpad)
    : SimpleTimingPort(_name, &_scratchpad), scratchpad(_scratchpad)
{ }

AddrRangeList
Scratchpad::ScratchpadPort::getAddrRanges() const
{
    AddrRangeList ranges;
    // everything until IO space
    ranges.push_back(AddrRange(0, 0x2000000000000000 - 1));
    return ranges;
}

Tick
Scratchpad::ScratchpadPort::recvAtomic(PacketPtr pkt)
{
    return scratchpad.recvAtomic(pkt);
}

Scratchpad*
ScratchpadParams::create()
{
    return new Scratchpad(this);
}
