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

#include "debug/Dtu.hh"
#include "mem/dtu/dtu.hh"

Dtu::Dtu(const DtuParams *p)
  : BaseDtu(p),
    cpu(*this),
    scratchpad(*this),
    master(*this),
    slave(*this)
{ }

void
Dtu::init()
{
    BaseDtu::init();

    assert(cpu.isConnected());
    assert(scratchpad.isConnected());
    assert(master.isConnected());
    assert(slave.isConnected());

    cpu.sendRangeChange();
    slave.sendRangeChange();

    // kick tings into action
    schedule(tickEvent, nextCycle());
}

BaseMasterPort&
Dtu::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "scratchpad")
        return scratchpad;
    else if (if_name == "master")
        return master;
    else
        return MemObject::getMasterPort(if_name, idx);
}

BaseSlavePort&
Dtu::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu")
        return cpu;
    else if (if_name == "slave")
        return slave;
    else
        return MemObject::getSlavePort(if_name, idx);
}

void
Dtu::sendSpmRequest(PacketPtr pkt)
{
    scratchpad.sendRequest(pkt);
}

void
Dtu::sendNocRequest(PacketPtr pkt)
{
    master.sendRequest(pkt);
}

void
Dtu::sendNocResponse(PacketPtr pkt, Cycles latency)
{
    slave.schedTimingResp(pkt, latency);
}

void
Dtu::sendCpuResponse(PacketPtr pkt, Cycles latency)
{
    cpu.schedTimingResp(pkt, latency);
}

bool
Dtu::isSpmPortReady()
{
    return scratchpad.isReady();
}

bool
Dtu::isNocPortReady()
{
    return master.isReady();
}

void
Dtu::DtuMasterPort::sendRequest(PacketPtr pkt)
{
    assert(reqPkt == nullptr);
    assert(waitForRetry == false);

    DPRINTF(Dtu, "Send %s request at address 0x%x (%u bytes)\n",
                 pkt->isRead() ? "read" : "write",
                 pkt->getAddr(),
                 pkt->getSize());

    if (dtu.atomic)
    {
        sendAtomic(pkt);
        completeRequest(pkt);
    }
    else
    {
        waitForRetry = !sendTimingReq(pkt);

        if (waitForRetry)
        {
            reqPkt = pkt;

            DPRINTF(Dtu, "Request failed. Wait for retry.\n");
        }
    }
}

void
Dtu::DtuMasterPort::recvReqRetry()
{
    assert(waitForRetry == true);
    assert(reqPkt != nullptr);

    if (sendTimingReq(reqPkt))
    {
        DPRINTF(Dtu, "Successful retry\n");

        waitForRetry = false;

        reqPkt = nullptr;
    }
}

bool
Dtu::DtuMasterPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(Dtu, "Received response %#x\n", pkt->getAddr());

    // Pay for the transport delay and schedule event on clock edge
    Tick delay = pkt->headerDelay + pkt->payloadDelay;

    pkt->headerDelay = 0;
    pkt->payloadDelay = 0;

    Tick tick = dtu.clockEdge(dtu.ticksToCycles(delay));

    auto dpkt = DeferredPacket(tick, pkt);

    respQueue.push(dpkt);

    dtu.schedule(responseEvent, tick);

    return true;
}

void
Dtu::DtuMasterPort::ResponseEvent::process()
{
    assert(!port.respQueue.empty());

    DeferredPacket dpkt = port.respQueue.front();

    port.respQueue.pop();

    port.completeRequest(dpkt.pkt);
}

void
Dtu::NocMasterPort::completeRequest(PacketPtr pkt)
{
    dtu.completeNocRequest(pkt);
}

void
Dtu::ScratchpadPort::completeRequest(PacketPtr pkt)
{
    dtu.completeSpmRequest(pkt);
}

void
Dtu::DtuSlavePort::schedTimingResp(PacketPtr pkt, Cycles latency)
{
    assert(busy);
    assert(respPkt == nullptr);
    assert(!waitForRetry);

    respPkt = pkt;

    dtu.schedule(responseEvent, dtu.clockEdge(latency));
}

Tick
Dtu::DtuSlavePort::recvAtomic(PacketPtr pkt)
{
    handleRequest(pkt);

    return 0;
}

void
Dtu::DtuSlavePort::recvFunctional(PacketPtr pkt)
{
    panic("%S does not implement recvFunctional!\n", name());
}

bool
Dtu::DtuSlavePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(Dtu, "Recieve timing request %#x\n", pkt->getAddr());

    if (busy)
    {
        sendRetry = true;
        return false;
    }

    assert(!sendRetry);
    assert(reqPkt == nullptr);

    busy = true;

    reqPkt = pkt;

    Tick delay = pkt->headerDelay + pkt->payloadDelay;

    pkt->headerDelay = 0;
    pkt->payloadDelay = 0;

    dtu.schedule(requestEvent, dtu.clockEdge(dtu.ticksToCycles(delay)));

    return true;
}

void
Dtu::DtuSlavePort::recvRespRetry()
{
    assert(respPkt != nullptr);
    assert(waitForRetry);

    if (SlavePort::sendTimingResp(respPkt))
    {
        busy = false;
        waitForRetry = false;
        respPkt = nullptr;

        if (sendRetry)
        {
            sendRetry = false;
            sendRetryReq();
        }
    }
}

void
Dtu::DtuSlavePort::RequestEvent::process()
{
    assert(port.reqPkt != nullptr);

    if (port.handleRequest(port.reqPkt))
        port.reqPkt = nullptr;
    else
        port.dtu.schedule(this, port.dtu.nextCycle());
}

void
Dtu::DtuSlavePort::ResponseEvent::process()
{
    assert(port.respPkt != nullptr);
    assert(!port.waitForRetry);

    if (port.sendTimingResp(port.respPkt))
    {
        port.busy = false;
        port.respPkt = nullptr;

        if (port.sendRetry)
        {
            port.sendRetry = false;
            port.sendRetryReq();
        }
    }
    else
    {
        port.waitForRetry = true;
    }
}

AddrRangeList
Dtu::CpuPort::getAddrRanges() const
{
    AddrRangeList ranges;
    auto range = AddrRange(dtu.cpuBaseAddr,
                           dtu.cpuBaseAddr + dtu.regFile.getSize() - 1);
    ranges.push_back(range);
    return ranges;
}

bool
Dtu::CpuPort::handleRequest(PacketPtr pkt)
{
    return dtu.handleCpuRequest(pkt);
}

AddrRangeList
Dtu::NocSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;

    // XXX we assume 64 bit addresses
    Addr size = 1UL << (64 - dtu.nocAddrBits);

    auto range = AddrRange(dtu.nocBaseAddr, dtu.nocBaseAddr + (size - 1));

    ranges.push_back(range);

    return ranges;
}

bool
Dtu::NocSlavePort::handleRequest(PacketPtr pkt)
{
    return dtu.handleNocRequest(pkt);
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}
