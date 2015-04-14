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
    cpu("cpu", *this),
    scratchpad("scratchpad", *this),
    master("master", *this),
    slave("slave", *this),
    retrySpmPkt(nullptr),
    retryNocPkt(nullptr),
    nocWaitsForRetry(false),
    tickEvent(this)
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
Dtu::recvSpmRetry()
{
    assert(retrySpmPkt);

    if (scratchpad.sendTimingReq(retrySpmPkt))
    {
        DPRINTF(Dtu, "Successfull retry on scratchpad port.\n");

        retrySpmPkt = nullptr;
    }
}

void
Dtu::recvNocRetry()
{
    assert(retryNocPkt);

    if (master.sendTimingReq(retryNocPkt))
    {
        DPRINTF(Dtu, "Successfull retry on NoC port.\n");

        retryNocPkt = nullptr;
    }
}

void
Dtu::sendSpmRequest(PacketPtr pkt)
{
    assert(retrySpmPkt == nullptr);

    DPRINTF(Dtu, "Send %s request to Scatchpad at address 0x%x (%u bytes)\n",
                 pkt->isRead() ? "read" : "write",
                 pkt->getAddr(),
                 pkt->getSize());

    if (atomic)
    {
        scratchpad.sendAtomic(pkt);
        completeSpmRequest(pkt);
    }
    else
    {
        bool retry = !scratchpad.sendTimingReq(pkt);

        if (retry)
        {
            DPRINTF(Dtu, "Request failed. Wait for retry\n");

            retrySpmPkt = pkt;
        }
    }
}

void
Dtu::sendNocRequest(PacketPtr pkt)
{
    assert(retryNocPkt == nullptr);

    DPRINTF(Dtu, "Send %s request to the NoC at address 0x%x (%u bytes)\n",
                 pkt->isRead() ? "read" : "write",
                 pkt->getAddr(),
                 pkt->getSize());

    if (atomic)
    {
        master.sendAtomic(pkt);
        completeNocRequest(pkt);
    }
    else
    {
        bool retry = !master.sendTimingReq(pkt);

        if (retry)
        {
            DPRINTF(Dtu, "Request failed. Wait for retry\n");

            retryNocPkt = pkt;
        }
    }
}

void
Dtu::sendNocResponse(PacketPtr pkt)
{
    slave.schedTimingResp(pkt, curTick());
}

bool
Dtu::isSpmPortReady()
{
    return retrySpmPkt == nullptr;
}

bool
Dtu::isNocPortReady()
{
    return retryNocPkt == nullptr;
}

void
Dtu::tick()
{
    BaseDtu::tick();

    if (nocWaitsForRetry && canHandleNocRequest())
    {
        nocWaitsForRetry = false;
        DPRINTF(Dtu, "Send NoC retry\n");
        slave.sendRetryReq();
    }

    schedule(tickEvent, nextCycle());
}

bool
Dtu::DtuScratchpadPort::recvTimingResp(PacketPtr pkt)
{
    // TODO We should pay somewhere for the delay caused by the
    //      transport layer.
    dtu.completeSpmRequest(pkt);
    return true;
}

void
Dtu::DtuScratchpadPort::recvReqRetry()
{
    dtu.recvSpmRetry();
}

AddrRangeList
Dtu::DtuCpuPort::getAddrRanges() const
{
    AddrRangeList ranges;
    auto range = AddrRange(dtu.cpuBaseAddr,
                           dtu.cpuBaseAddr + dtu.regFile.getSize() - 1);
    ranges.push_back(range);
    return ranges;
}

Tick
Dtu::DtuCpuPort::recvAtomic(PacketPtr pkt)
{
    return dtu.handleCpuRequest(pkt);
}

bool
Dtu::DtuMasterPort::recvTimingResp(PacketPtr pkt)
{
    // TODO We should pay somewhere for the delay caused by the
    //      transport layer.
    dtu.completeNocRequest(pkt);

    return true;
}

void
Dtu::DtuMasterPort::recvReqRetry()
{
    dtu.recvNocRetry();
}

AddrRangeList
Dtu::DtuSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;

    // XXX we assume 64 bit addresses
    Addr size = 1UL << (64 - dtu.nocAddrBits);

    auto range = AddrRange(dtu.nocBaseAddr, dtu.nocBaseAddr + (size - 1));

    ranges.push_back(range);

    return ranges;
}

Tick
Dtu::DtuSlavePort::recvAtomic(PacketPtr pkt)
{
    return dtu.handleNocRequest(pkt);
}

void
Dtu::DtuSlavePort::recvFunctional(PacketPtr pkt)
{
    panic("Dtu::recvFunctional() not yet implemented");
}

bool
Dtu::DtuSlavePort::recvTimingReq(PacketPtr pkt)
{
    // We don't expect packets that do not need a response. Otherwise we would
    // need to delete handled packets.
    assert(pkt->needsResponse());

    if (!dtu.canHandleNocRequest())
    {
        dtu.nocWaitsForRetry = true;
        return false;
    }

    dtu.handleNocRequest(pkt);

    return true;
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}
