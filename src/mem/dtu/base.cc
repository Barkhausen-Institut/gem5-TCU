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

#include "mem/dtu/base.hh"

BaseDtu::DtuMasterPort::DtuMasterPort( const std::string& _name, BaseDtu& _dtu)
  : QueuedMasterPort(_name, &_dtu, reqQueue, snoopRespQueue),
    dtu(_dtu),
    reqQueue(_dtu, *this),
    snoopRespQueue(_dtu, *this)
{ }

bool
BaseDtu::DtuMasterPort::recvTimingResp(PacketPtr pkt)
{
    // If we receive a response we should always expect it
    completeRequest(pkt);

    return true;
}

void
BaseDtu::NocMasterPort::completeRequest(PacketPtr pkt)
{
    dtu.completeNocRequest(pkt);
}

void
BaseDtu::ScratchpadPort::completeRequest(PacketPtr pkt)
{
    dtu.completeSpmRequest(pkt);
}

BaseDtu::DtuSlavePort::DtuSlavePort(const std::string _name, BaseDtu& _dtu)
  : QueuedSlavePort(_name, &_dtu, respQueue),
    dtu(_dtu),
    respQueue(_dtu, *this),
    busy(false),
    retry(false)
{ }

Tick
BaseDtu::DtuSlavePort::recvAtomic(PacketPtr pkt)
{
    handleRequest(pkt);

    return 0;
}

void
BaseDtu::DtuSlavePort::recvFunctional(PacketPtr pkt)
{
    panic("%s did not expect a funtional request!\n");
}

bool
BaseDtu::DtuSlavePort::recvTimingReq(PacketPtr pkt)
{
    if (busy)
    {
        retry = true;
        return false;
    }

    handleRequest(pkt);

    return true;
}

void
BaseDtu::DtuSlavePort::setBusy()
{
    assert(!busy);

    busy = true;
}

void
BaseDtu::DtuSlavePort::setIdle()
{
    assert(busy);

    busy = false;

    if (retry)
    {
        retry = false;
        sendRetryReq();
    }
}

AddrRangeList
BaseDtu::CpuPort::getAddrRanges() const
{
    AddrRangeList ranges;

    auto range = AddrRange(dtu.cpuBaseAddr,
                           dtu.cpuBaseAddr + (dtu.cpuBaseAddr - 1));

    ranges.push_back(range);

    return ranges;
}

void
BaseDtu::CpuPort::handleRequest(PacketPtr pkt)
{
    dtu.handleCpuRequest(pkt);
}

AddrRangeList
BaseDtu::NocSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;

    Addr nocAddr = dtu.getNocAddr(dtu.coreId);

    auto range = AddrRange(nocAddr, nocAddr + (nocAddr - 1));

    ranges.push_back(range);

    return ranges;
}

BaseDtu::BaseDtu(BaseDtuParams* p)
  : MemObject(p),
    cpuPort(*this),
    scratchpadPort(*this),
    nocMasterPort(*this),
    nocSlavePort(*this),
    coreId(p->core_id),
    cpuBaseAddr(p->cpu_base_addr),
    nocAddrBits(p->noc_addr_bits)
{}

void
BaseDtu::NocSlavePort::handleRequest(PacketPtr pkt)
{
    dtu.handleNocRequest(pkt);
}

BaseMasterPort&
BaseDtu::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "scratchpad")
        return scratchpadPort;
    else if (if_name == "noc_master")
        return nocMasterPort;
    else
        return MemObject::getMasterPort(if_name, idx);
}

BaseSlavePort&
BaseDtu::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu")
        return cpuPort;
    else if (if_name == "noc_slave")
        return nocSlavePort;
    else
        return MemObject::getSlavePort(if_name, idx);
}

Addr
BaseDtu::getNocAddr(unsigned coreId) const
{
    return (static_cast<Addr>(coreId) << (sizeof(Addr) * 8 - nocAddrBits));
}

void
BaseDtu::schedNocResponse(PacketPtr pkt, Tick when)
{
    nocSlavePort.schedTimingResp(pkt, when);
}

void
BaseDtu::schedCpuResponse(PacketPtr pkt, Tick when)
{
    cpuPort.schedTimingResp(pkt, when);
}

void
BaseDtu::schedNocRequest(PacketPtr pkt, Tick when)
{
    nocMasterPort.schedTimingReq(pkt, when);
}

void
BaseDtu::schedSpmRequest(PacketPtr pkt, Tick when)
{
    scratchpadPort.schedTimingReq(pkt, when);

}

void
BaseDtu::sendAtomicNocRequest(PacketPtr pkt)
{
    nocMasterPort.sendAtomic(pkt);
}

void
BaseDtu::sendAtomicSpmRequest(PacketPtr pkt)
{
    scratchpadPort.sendAtomic(pkt);
}

void
BaseDtu::setCpuPortBusy()
{
    cpuPort.setBusy();
}

void
BaseDtu::setCpuPortIdle()
{
    cpuPort.setIdle();
}

void
BaseDtu::setNocPortBusy()
{
    nocSlavePort.setBusy();
}

void
BaseDtu::setNocPortIdle()
{
    nocSlavePort.setIdle();
}
