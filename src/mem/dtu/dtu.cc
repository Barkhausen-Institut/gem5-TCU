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
#include "sim/system.hh"

Dtu::Dtu(const DtuParams *p)
  : BaseDtu(p),
    cpu("cpu", *this),
    scratchpad("scratchpad", *this),
    master("master", *this),
    slave("slave", *this),
    atomic(p->system->isAtomicMode()),
    retrySpmPkt(nullptr)
{ }

void
Dtu::init()
{
    MemObject::init();

    assert(cpu.isConnected());
    assert(scratchpad.isConnected());
    assert(master.isConnected());
    assert(slave.isConnected());

    cpu.sendRangeChange();
    slave.sendRangeChange();
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
    assert(waitingForSpmRetry);

    if (scratchpad.sendTimingReq(retrySpmPkt))
    {
        DPRINTF(Dtu, "Wake up after successfull retry on scratchpad port.\n");

        retrySpmPkt = nullptr;

        waitingForSpmRetry = false;
    }
}

bool
Dtu::sendSpmRequest(PacketPtr pkt)
{
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

            waitingForSpmRetry = true;
            retrySpmPkt = pkt;

            return false;
        }
    }

    return true;
}

bool
Dtu::sendNocRequest(PacketPtr pkt)
{
    panic("Send to NoC it not yet implemented");

    return true;
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
    panic("Did not expect a TimingResp!");
    return true;
}

void
Dtu::DtuMasterPort::recvReqRetry()
{
    panic("Did not expect a ReqRetry!");
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
    panic("Dtu::recvAtomic() not yet implemented");
    return 0;
}

void
Dtu::DtuSlavePort::recvFunctional(PacketPtr pkt)
{
    panic("Dtu::recvFunctional() not yet implemented");
}

bool
Dtu::DtuSlavePort::recvTimingReq(PacketPtr pkt)
{
    panic("Dtu::recvTimingReq() not yet implemented");
}

void
Dtu::DtuSlavePort::recvRespRetry()
{
    panic("Dtu::recvRespRetry() not yet implemented");
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}
