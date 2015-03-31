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

    /*
     * TODO
     * So far the Scratchpad has no busy state -> it accepts all requests!
     */

    /*
     * TODO
     * Latency calculation does not depend on packet size. This works as long
     * as the Scratchpad is connected via a XBar, as the XBar calculates the
     * payloadDelay and therefore limits throughput. However, this does not take
     * into account slower memories or direct connections (without XBar).
     */
    pkt->headerDelay += latency * clockPeriod();

    Tick totalDelay = pkt->headerDelay + pkt->payloadDelay;

    /*
     * The SimpleTimingPort already pays for the delay returned by recvAtomic
     *  -> reset the packet delay
     *
     * XXX I'm not sure if this is the right way to go. However, it seems
     *     better than simply ignoring the packet's delays as it is doen for 
     *     instance in SimpleMemory.
     */
    pkt->headerDelay  = 0;
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
