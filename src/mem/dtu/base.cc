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
#include "debug/MemoryWatch.hh"
#include "debug/Dtu.hh"
#include "debug/DtuTlb.hh"
#include "mem/dtu/base.hh"
#include "mem/dtu/noc_addr.hh"

BaseDtu::DtuMasterPort::DtuMasterPort(const std::string& _name, BaseDtu& _dtu)
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
    // the DTU does never send requests to the icache. so just pass it back to
    // the CPU
    return dtu.icacheSlavePort.sendTimingResp(pkt);
}

bool
BaseDtu::DCacheMasterPort::recvTimingResp(PacketPtr pkt)
{
    // if there is a context-id and thread-id, the request came from the CPU
    if (pkt->req->hasContextId())
        return dtu.dcacheSlavePort.sendTimingResp(pkt);

    // otherwise from the DTU
    dtu.completeMemRequest(pkt);
    return true;
}

BaseDtu::DtuSlavePort::DtuSlavePort(const std::string& _name, BaseDtu& _dtu)
  : SlavePort(_name, &_dtu),
    dtu(_dtu),
    busy(false),
    sendReqRetry(false),
    pendingResponses()
{ }

void
BaseDtu::DtuSlavePort::requestFinished()
{
    assert(busy);
    busy = false;

    DPRINTF(DtuSlavePort, "Timing request finished\n");

    if (sendReqRetry)
    {
        DPRINTF(DtuSlavePort, "Send request retry\n");

        sendReqRetry = false;
        sendRetryReq();
    }
}

void
BaseDtu::DtuSlavePort::schedTimingResp(PacketPtr pkt, Tick when)
{
    DPRINTF(DtuSlavePort, "Schedule timing response %#x at Tick %u\n",
                          pkt->getAddr(),
                          when);

    assert(pkt->isResponse());

    auto respEvent = new ResponseEvent(*this, pkt);
    dtu.schedule(respEvent, when);
}

Tick
BaseDtu::DtuSlavePort::recvAtomic(PacketPtr pkt)
{
    DPRINTF(DtuSlavePort, "Receive atomic %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

    handleRequest(pkt, &busy, false);

    return 0;
}

void
BaseDtu::DtuSlavePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(DtuSlavePort, "Receive functional %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

    // don't actually make us busy/unbusy here, because we might interfere
    // with the timing requests
    bool dummy = false;
    handleRequest(pkt, &dummy, true);
}

bool
BaseDtu::DtuSlavePort::recvTimingReq(PacketPtr pkt)
{
    if (busy)
    {
        DPRINTF(DtuSlavePort, "Reject timing %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

        sendReqRetry = true;
        return false;
    }

    DPRINTF(DtuSlavePort, "Receive timing %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

    assert(!sendReqRetry);

    return handleRequest(pkt, &busy, false);
}

void
BaseDtu::DtuSlavePort::recvRespRetry()
{
    // try to send all queued responses. the first one should always succeed
    // because the XBar called us because it is free. the second should always
    // fail since it is busy then. in this case, we stop here.
    while (!pendingResponses.empty())
    {
        ResponseEvent *ev = pendingResponses.front();

        DPRINTF(DtuSlavePort, "Receive response retry at %#x\n",
                              ev->pkt->getAddr());

        if (sendTimingResp(ev->pkt))
        {
            DPRINTF(DtuSlavePort, "Resume after successful retry at %#x\n",
                                  ev->pkt->getAddr());

            DPRINTF(DtuSlavePort, "Poping %p from queue\n", ev);
            pendingResponses.pop();
            delete ev;
        }
        else
            break;
    }
}

void
BaseDtu::DtuSlavePort::ResponseEvent::process()
{
    // if the XBar is busy, we can only send a response to a port once. this
    // is queued there and calls our recvRespRetry() if the XBar is idle again.
    // but we can't try another time. so, for the second time, don't even try
    // but directly queue it here
    if (port.pendingResponses.empty())
    {
        DPRINTF(DtuSlavePort, "Try to send %s response at %#x (%u bytes)\n",
                              pkt->cmd.toString(),
                              pkt->getAddr(),
                              pkt->getSize());

        if (!port.sendTimingResp(pkt))
        {
            DPRINTFS(DtuSlavePort, (&port), "Pushing %p to queue\n", this);
            port.pendingResponses.push(this);
        }
        // if it succeeded, let the event system delete the event
        else
            setFlags(AutoDelete);
    }
    else
    {
        DPRINTFS(DtuSlavePort, (&port), "Pushing %p to queue\n", this);
        port.pendingResponses.push(this);
    }
}

AddrRangeList
BaseDtu::NocSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;

    Addr baseNocAddr = NocAddr(dtu.coreId, 0, 0).getAddr();
    Addr topNocAddr  = NocAddr(dtu.coreId + 1, 0, 0).getAddr() - 1;

    DPRINTF(DtuSlavePort, "Dtu %u covers %#x to %#x\n",
                          dtu.coreId,
                          baseNocAddr,
                          topNocAddr);

    auto range = AddrRange(baseNocAddr, topNocAddr);

    ranges.push_back(range);

    return ranges;
}

AddrRangeList
BaseDtu::CacheMemSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;

    auto range = AddrRange(0, -1);

    ranges.push_back(range);

    return ranges;
}

BaseDtu::BaseDtu(BaseDtuParams* p)
  : MemObject(p),
    nocMasterPort(*this),
    nocSlavePort(*this),
    icacheMasterPort(*this),
    dcacheMasterPort(*this),
    icacheSlavePort(icacheMasterPort, *this, true),
    dcacheSlavePort(dcacheMasterPort, *this, false),
    cacheMemSlavePort(*this),
    watchRange(1, 0),
    nocReqFinishedEvent(*this),
    coreId(p->core_id),
    regFileBaseAddr(p->regfile_base_addr)
{
    if (p->watch_range_start != p->watch_range_end)
        watchRange = AddrRange(p->watch_range_start, p->watch_range_end - 1);
}

void
BaseDtu::init()
{
    MemObject::init();

    assert(nocMasterPort.isConnected());
    assert(nocSlavePort.isConnected());

    nocSlavePort.sendRangeChange();

    // for memory-PEs, the icache/dcache slaves are not connected
    if (icacheSlavePort.isConnected())
        icacheSlavePort.sendRangeChange();
    if (dcacheSlavePort.isConnected())
        dcacheSlavePort.sendRangeChange();

    // the cache-mem slave port is only used if we have a cache
    if (cacheMemSlavePort.isConnected())
        cacheMemSlavePort.sendRangeChange();
}

bool
BaseDtu::NocSlavePort::handleRequest(PacketPtr pkt,
                                     bool *busy,
                                     bool functional)
{
    *busy = true;

    dtu.handleNocRequest(pkt);

    return true;
}

bool
BaseDtu::CacheMemSlavePort::handleRequest(PacketPtr pkt,
                                          bool *busy,
                                          bool functional)
{
    // if that failed, it was an invalid request (probably due to speculative
    // execution)
    if (!dtu.handleCacheMemRequest(pkt, functional))
        dtu.sendDummyResponse(*this, pkt, functional);

    // in general, pretend that everything is fine
    return true;
}

void
BaseDtu::sendDummyResponse(DtuSlavePort &port, PacketPtr pkt, bool functional)
{
    // invalid reads just get zeros
    if (pkt->isRead())
        memset(pkt->getPtr<uint8_t>(), 0, pkt->getSize());

    // if a response is necessary, send one
    if (pkt->needsResponse())
    {
        pkt->makeResponse();

        if (!functional)
        {
            // somehow we need to send that later to make the cache happy.
            port.schedTimingResp(pkt, clockEdge(Cycles(1)));
        }
    }
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
    else if (if_name == "cache_mem_slave_port")
        return cacheMemSlavePort;
    else
        return MemObject::getSlavePort(if_name, idx);
}

void
BaseDtu::checkWatchRange(PacketPtr pkt)
{
    if (watchRange.valid())
    {
        AddrRange range(pkt->getAddr(), pkt->getAddr() + pkt->getSize() - 1);
        if (watchRange.intersects(range))
        {
            DPRINTF(MemoryWatch,
                "%s access to address range %p..%p (watching %p..%p)\n",
                pkt->isRead() ? "read" : "write",
                range.start(), range.end(),
                watchRange.start(), watchRange.end());
            DDUMP(MemoryWatch, pkt->getPtr<uint8_t>(), pkt->getSize());
        }
    }
}

void
BaseDtu::schedNocResponse(PacketPtr pkt, Tick when)
{
    assert(pkt->isResponse());

    nocSlavePort.schedTimingResp(pkt, when);
}

void
BaseDtu::schedNocRequestFinished(Tick when)
{
    schedule(nocReqFinishedEvent, when);
}

void
BaseDtu::nocRequestFinished()
{
    nocSlavePort.requestFinished();
}

void
BaseDtu::schedCpuResponse(PacketPtr pkt, Tick when)
{
    assert(pkt->isResponse());

    dcacheSlavePort.schedTimingResp(pkt, when);
}

void
BaseDtu::sendCacheMemResponse(PacketPtr pkt, bool success)
{
    DPRINTF(DtuSlavePort, "Send %s response at %#x (%u bytes)\n",
            pkt->cmd.toString(),
            pkt->getAddr(),
            pkt->getSize());

    if (!success)
        sendDummyResponse(cacheMemSlavePort, pkt, false);
    else
        cacheMemSlavePort.sendTimingResp(pkt);
}

void
BaseDtu::schedNocRequest(PacketPtr pkt, Tick when)
{
    nocMasterPort.schedTimingReq(pkt, when);
}

void
BaseDtu::schedMemRequest(PacketPtr pkt, Tick when)
{
    checkWatchRange(pkt);

    dcacheMasterPort.schedTimingReq(pkt, when);
}

void
BaseDtu::sendFunctionalNocRequest(PacketPtr pkt)
{
    nocMasterPort.sendFunctional(pkt);
}

void
BaseDtu::sendAtomicNocRequest(PacketPtr pkt)
{
    nocMasterPort.sendAtomic(pkt);
}

void
BaseDtu::sendAtomicMemRequest(PacketPtr pkt)
{
    checkWatchRange(pkt);

    dcacheMasterPort.sendAtomic(pkt);
}
