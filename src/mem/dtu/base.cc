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

#include "debug/DtuSlavePort.hh"
#include "debug/DtuMasterPort.hh"
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

bool
BaseDtu::ICacheMasterPort::recvTimingResp(PacketPtr pkt)
{
    // the DTU does never send requests to the icache. so just pass it back to the CPU
    return dtu.icacheSlavePort.sendTimingResp(pkt);
}

bool
BaseDtu::DCacheMasterPort::recvTimingResp(PacketPtr pkt)
{
    // if there is a context-id and thread-id, the request came from the CPU
    if(pkt->req->hasContextId())
        return dtu.dcacheSlavePort.sendTimingResp(pkt);

    // otherwise from the DTU
    dtu.completeSpmRequest(pkt);
    return true;
}

BaseDtu::DtuSlavePort::DtuSlavePort(const std::string& _name, BaseDtu& _dtu)
  : SlavePort(_name, &_dtu),
    dtu(_dtu),
    busy(false),
    sendReqRetry(false),
    respPkt(nullptr),
    waitForRespRetry(false),
    responseEvent(*this)
{ }

void
BaseDtu::DtuSlavePort::schedTimingResp(PacketPtr pkt, Tick when)
{
    assert(busy);
    assert(respPkt == nullptr);
    assert(!waitForRespRetry);

    respPkt = pkt;

    DPRINTF(DtuSlavePort, "Schedule response %#x at Tick %u\n",
                          pkt->getAddr(),
                          when);

    dtu.schedule(responseEvent, when);
}

Tick
BaseDtu::DtuSlavePort::recvAtomic(PacketPtr pkt)
{
    DPRINTF(DtuSlavePort, "Receive %s request at %#x (%u bytes)\n",
                          pkt->isRead() ? "read" : "write",
                          pkt->getAddr(),
                          pkt->getSize());

    handleRequest(pkt, &busy, false);

    return 0;
}

void
BaseDtu::DtuSlavePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(DtuSlavePort, "Receive %s request at %#x (%u bytes)\n",
                          pkt->isRead() ? "read" : "write",
                          pkt->getAddr(),
                          pkt->getSize());

    handleRequest(pkt, &busy, true);
}

bool
BaseDtu::DtuSlavePort::recvTimingReq(PacketPtr pkt)
{
    if (busy)
    {
        DPRINTF(DtuSlavePort, "Reject %s request at %#x (%u bytes)\n",
                          pkt->isRead() ? "read" : "write",
                          pkt->getAddr(),
                          pkt->getSize());

        sendReqRetry = true;
        return false;
    }

    DPRINTF(DtuSlavePort, "Receive %s request at %#x (%u bytes)\n",
                          pkt->isRead() ? "read" : "write",
                          pkt->getAddr(),
                          pkt->getSize());

    assert(!sendReqRetry);

    return handleRequest(pkt, &busy, false);
}

void
BaseDtu::DtuSlavePort::handleSuccessfulResponse()
{
    busy = false;
    waitForRespRetry = false;
    respPkt = nullptr;

    if (sendReqRetry)
    {
        DPRINTF(DtuSlavePort, "Send request retry\n");

        sendReqRetry = false;
        sendRetryReq();
    }
}

void
BaseDtu::DtuSlavePort::recvRespRetry()
{
    assert(respPkt != nullptr);
    assert(busy);
    assert(waitForRespRetry);

    DPRINTF(DtuSlavePort, "Receive response retry at %#x\n", respPkt->getAddr());

    if (sendTimingResp(respPkt))
    {
        DPRINTF(DtuSlavePort, "Resume after successful retry at %#x\n",
                              respPkt->getAddr());

        handleSuccessfulResponse();
    }
}

void
BaseDtu::DtuSlavePort::ResponseEvent::process()
{
    assert(port.busy);
    assert(port.respPkt != nullptr);
    assert(!port.waitForRespRetry);

    DPRINTF(DtuSlavePort, "Send %s response at %#x (%u bytes)\n",
                          port.respPkt->isRead() ? "read" : "write",
                          port.respPkt->getAddr(),
                          port.respPkt->getSize());

    if (port.sendTimingResp(port.respPkt))
        port.handleSuccessfulResponse();
    else
        port.waitForRespRetry = true;
}

AddrRangeList
BaseDtu::NocSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;

    Addr baseNocAddr = dtu.getNocAddr(dtu.coreId);
    Addr topNocAddr  = dtu.getNocAddr(dtu.coreId + 1) - 1;

    DPRINTF(DtuSlavePort, "Dtu %u covers %#x to %#x\n", dtu.coreId, baseNocAddr, topNocAddr);

    auto range = AddrRange(baseNocAddr, topNocAddr);

    ranges.push_back(range);

    return ranges;
}

BaseDtu::BaseDtu(BaseDtuParams* p)
  : MemObject(p),
    nocMasterPort(*this),
    nocSlavePort(*this),
    icacheMasterPort(*this),
    dcacheMasterPort(*this),
    icacheSlavePort(icacheMasterPort, *this),
    dcacheSlavePort(dcacheMasterPort, *this),
    coreId(p->core_id),
    regFileBaseAddr(p->regfile_base_addr),
    nocAddrWidth(p->noc_addr_width),
    nocCoreAddrBits(p->noc_core_addr_bits),
    nocEpAddrBits(p->noc_ep_addr_bits)
{}

void
BaseDtu::init()
{
    MemObject::init();

    assert(nocMasterPort.isConnected());
    assert(nocSlavePort.isConnected());

    nocSlavePort.sendRangeChange();

    // for memory-PEs, the icache/dcache slaves are not connected
    if(icacheSlavePort.isConnected())
        icacheSlavePort.sendRangeChange();
    if(dcacheSlavePort.isConnected())
        dcacheSlavePort.sendRangeChange();
}

bool
BaseDtu::NocSlavePort::handleRequest(PacketPtr pkt, bool *busy, bool functional)
{
    assert(!functional);

    *busy = true;

    dtu.handleNocRequest(pkt);
    return true;
}

BaseMasterPort&
BaseDtu::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "icache_master_port")
        return icacheMasterPort;
    else if (if_name == "dcache_master_port")
        return dcacheMasterPort;
    else if (if_name == "noc_master_port")
        return nocMasterPort;
    else
        return MemObject::getMasterPort(if_name, idx);
}

BaseSlavePort&
BaseDtu::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name == "icache_slave_port")
        return icacheSlavePort;
    else if (if_name == "dcache_slave_port")
        return dcacheSlavePort;
    else if (if_name == "noc_slave_port")
        return nocSlavePort;
    else
        return MemObject::getSlavePort(if_name, idx);
}

Addr
BaseDtu::getNocAddr(unsigned coreId, unsigned epId) const
{
    /*
     * nocAddrWidth - 1                     0
     *        -------------------------------
     *        | 1 |  coreId  | ... |  epId  |
     *        -------------------------------
     */
    assert(nocCoreAddrBits + nocEpAddrBits + 1 <= nocAddrWidth);

    // MSB is always 1
    Addr res = static_cast<Addr>(1) << (nocAddrWidth - 1);

    res |= static_cast<Addr>(coreId) << (nocAddrWidth  - nocCoreAddrBits - 1);

    res |= epId & ((1 << nocEpAddrBits) - 1);

    return res;
}

void
BaseDtu::schedNocResponse(PacketPtr pkt, Tick when)
{
    nocSlavePort.schedTimingResp(pkt, when);
}

void
BaseDtu::schedCpuResponse(PacketPtr pkt, Tick when)
{
    dcacheSlavePort.schedTimingResp(pkt, when);
}

void
BaseDtu::schedNocRequest(PacketPtr pkt, Tick when)
{
    nocMasterPort.schedTimingReq(pkt, when);
}

void
BaseDtu::schedSpmRequest(PacketPtr pkt, Tick when)
{
    dcacheMasterPort.schedTimingReq(pkt, when);
}

void
BaseDtu::sendAtomicNocRequest(PacketPtr pkt)
{
    nocMasterPort.sendAtomic(pkt);
}

void
BaseDtu::sendAtomicSpmRequest(PacketPtr pkt)
{
    dcacheMasterPort.sendAtomic(pkt);
}
