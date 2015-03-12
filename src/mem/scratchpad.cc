/*
 * Copyright (c) 2015, Christian Menard
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

#include "mem/scratchpad.hh"

Scratchpad::Scratchpad(const ScratchpadParams* p)
  : AbstractMemory(p),
    port(name() + ".port", *this),
    latency(p->latency)
{
}

void
Scratchpad::init()
{
    AbstractMemory::init();

    assert(port.isConnected());

    port.sendRangeChange();
}

BaseSlavePort &
Scratchpad::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name != "port") {
        return MemObject::getSlavePort(if_name, idx);
    } else {
        return port;
    }
}

Tick
Scratchpad::recvAtomic(PacketPtr pkt)
{
    access(pkt);
    return latency;
}

void
Scratchpad::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    functionalAccess(pkt);

    pkt->popLabel();
}

bool
Scratchpad::recvTimingReq(PacketPtr pkt)
{
    panic("Scratchpad::recvTimingReq() not yet implemented");
}

void
Scratchpad::recvRespRetry()
{
    panic("Scratchpad::recvRespRetry() not yet implemented");
}


Scratchpad::ScratchpadPort::ScratchpadPort(const std::string& _name,
                                           Scratchpad& _scratchpad)
    : SlavePort(_name, &_scratchpad), scratchpad(_scratchpad)
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
    scratchpad.recvFunctional(pkt);
}

bool
Scratchpad::ScratchpadPort::recvTimingReq(PacketPtr pkt)
{
    return scratchpad.recvTimingReq(pkt);
}

void
Scratchpad::ScratchpadPort::recvRespRetry()
{
    scratchpad.recvRespRetry();
}

Scratchpad*
ScratchpadParams::create()
{
    return new Scratchpad(this);
}
