/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (c) 2015, Nils Asmussen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 */

#include "debug/MemoryWatch.hh"
#include "mem/scratchpad.hh"

Scratchpad::Scratchpad(const ScratchpadParams* p)
  : AbstractMemory(p),
    cpuPort(name() + ".cpu_port", *this),
    dtuPort(name() + ".dtu_port", *this),
    latency(p->latency),
    throughput(p->throughput),
    initFile(p->init_file),
    watchRange(1, 0)
{
    if(p->watch_range_start != p->watch_range_end)
        watchRange = AddrRange(p->watch_range_start, p->watch_range_end - 1);
}

void
Scratchpad::init()
{
    AbstractMemory::init();

    assert(cpuPort.isConnected());
    // assert(dtuPort.isConnected());

    cpuPort.sendRangeChange();
    if(dtuPort.isConnected())
        dtuPort.sendRangeChange();

    if(!initFile.empty()) {
        FILE *f = fopen(initFile.c_str(), "r");
        if(!f)
            panic("Unable to open '%s' for reading", initFile.c_str());

        fseek(f, 0L, SEEK_END);
        size_t sz = ftell(f);
        fseek(f, 0L, SEEK_SET);

        Request::Flags flags;
        auto req = new Request(0, sz, flags, 0);
        auto pkt = new Packet(req, MemCmd::WriteReq);
        auto pktData = new uint8_t[sz];
        fread(pktData, 1, sz, f);
        pkt->dataDynamic(pktData);
        fclose(f);

        this->functionalAccess(pkt);

        delete pkt->req;
        delete pkt;
    }
}

BaseSlavePort &
Scratchpad::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_port") {
        return cpuPort;
    }
    else if(if_name == "dtu_port") {
        return dtuPort;
    }
    else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

Tick
Scratchpad::recvAtomic(PacketPtr pkt)
{
    /*
     * TODO
     * So far the Scratchpad has no busy state -> it accepts all requests!
     */

    // let subclass handle the request
    access(pkt);

    if(watchRange.valid())
    {
        AddrRange range(pkt->getAddr(), pkt->getAddr() + pkt->getSize() - 1);
        if(watchRange.intersects(range))
        {
            DPRINTF(MemoryWatch, "%s access to address range %p..%p (watching %p..%p)\n",
                pkt->isRead() ? "read" : "write", range.start(), range.end(),
                watchRange.start(), watchRange.end());
            DDUMP(MemoryWatch, pkt->getPtr<uint8_t>(), pkt->getSize());
        }
    }

    unsigned cyclesNeeded = round(((float) pkt->getSize()) / ((float)throughput));
    unsigned payloadDelay = clockPeriod() * cyclesNeeded;

    // pay the header delay caused by interconnect and add the scratchpad latency
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
    ranges.push_back(scratchpad.getAddrRange());
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
