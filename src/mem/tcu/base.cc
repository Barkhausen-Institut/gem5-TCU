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

#include "debug/TcuSlavePort.hh"
#include "debug/TcuMasterPort.hh"
#include "debug/Tcu.hh"
#include "mem/tcu/base.hh"
#include "mem/tcu/noc_addr.hh"

BaseTcu::TcuMasterPort::TcuMasterPort(const std::string& _name, BaseTcu& _tcu)
  : QueuedMasterPort(_name, &_tcu, reqQueue, snoopRespQueue),
    tcu(_tcu),
    reqQueue(_tcu, *this),
    snoopRespQueue(_tcu, *this)
{ }

bool
BaseTcu::TcuMasterPort::recvTimingResp(PacketPtr pkt)
{
    // If we receive a response we should always expect it
    completeRequest(pkt);

    return true;
}

void
BaseTcu::NocMasterPort::completeRequest(PacketPtr pkt)
{
    DPRINTF(TcuMasterPort,
            "Received %s at %#x (%u bytes)\n",
            pkt->cmd.toString(),
            pkt->getAddr(),
            pkt->getSize());

    tcu.completeNocRequest(pkt);
}

void
BaseTcu::NocMasterPort::recvFunctionalSnoop(PacketPtr pkt)
{
    if (tcu.cacheMemSlavePort.isConnected())
        tcu.cacheMemSlavePort.sendFunctionalSnoop(pkt);
}

void
BaseTcu::NocMasterPort::recvTimingSnoopReq(PacketPtr pkt)
{
    if (tcu.cacheMemSlavePort.isConnected())
        tcu.cacheMemSlavePort.sendTimingSnoopReq(pkt);
}

void
BaseTcu::NocMasterPort::recvRetrySnoopResp()
{
    if (tcu.cacheMemSlavePort.isConnected())
        tcu.cacheMemSlavePort.sendRetrySnoopResp();
}

bool
BaseTcu::CacheMemSlavePort::recvTimingSnoopResp(PacketPtr pkt)
{
    return tcu.nocMasterPort.sendTimingSnoopResp(pkt);
}

bool
BaseTcu::ICacheMasterPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(TcuSlavePort, "Sending timing response at %#x [senderState=%#x]\n",
                          pkt->getAddr(),
                          pkt->senderState);

    // the TCU does never send requests to the icache. so just pass it back to
    // the CPU
    tcu.icacheSlavePort.schedTimingResp(pkt, tcu.clockEdge(Cycles(1)));
    return true;
}

bool
BaseTcu::DCacheMasterPort::recvTimingResp(PacketPtr pkt)
{
    // if there is a context-id and thread-id, the request came from the CPU
    if (pkt->req->hasContextId())
    {
        DPRINTF(TcuSlavePort, "Sending timing response at %#x [senderState=%#x]\n",
                              pkt->getAddr(),
                              pkt->senderState);

        tcu.dcacheSlavePort.schedTimingResp(pkt, tcu.clockEdge(Cycles(1)));
        return true;
    }

    // otherwise from the TCU
    tcu.completeMemRequest(pkt);
    return true;
}

BaseTcu::TcuSlavePort::TcuSlavePort(const std::string& _name, BaseTcu& _tcu)
  : SlavePort(_name, &_tcu),
    tcu(_tcu),
    busy(false),
    sendReqRetry(false),
    pendingResponses()
{ }

void
BaseTcu::TcuSlavePort::requestFinished()
{
    assert(busy);
    busy = false;

    DPRINTF(TcuSlavePort, "Timing request finished\n");

    if (sendReqRetry)
    {
        DPRINTF(TcuSlavePort, "Send request retry\n");

        sendReqRetry = false;
        sendRetryReq();
    }
}

void
BaseTcu::TcuSlavePort::schedTimingResp(PacketPtr pkt, Tick when)
{
    DPRINTF(TcuSlavePort, "Schedule timing response %#x at Tick %u\n",
                          pkt->getAddr(),
                          when);

    assert(pkt->isResponse());

    auto respEvent = new ResponseEvent(*this, pkt);
    tcu.schedule(respEvent, when);
}

Tick
BaseTcu::TcuSlavePort::recvAtomic(PacketPtr pkt)
{
    DPRINTF(TcuSlavePort, "Receive atomic %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

    handleRequest(pkt, &busy, false);

    return 0;
}

void
BaseTcu::TcuSlavePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(TcuSlavePort, "Receive functional %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

    // don't actually make us busy/unbusy here, because we might interfere
    // with the timing requests
    bool dummy = false;
    handleRequest(pkt, &dummy, true);
}

bool
BaseTcu::TcuSlavePort::recvTimingReq(PacketPtr pkt)
{
    if (busy)
    {
        DPRINTF(TcuSlavePort, "Reject timing %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

        sendReqRetry = true;
        return false;
    }

    DPRINTF(TcuSlavePort, "Receive timing %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

    assert(!sendReqRetry);

    return handleRequest(pkt, &busy, false);
}

void
BaseTcu::TcuSlavePort::recvRespRetry()
{
    // try to send all queued responses. the first one should always succeed
    // because the XBar called us because it is free. the second should always
    // fail since it is busy then. in this case, we stop here.
    while (!pendingResponses.empty())
    {
        ResponseEvent *ev = pendingResponses.front();

        DPRINTF(TcuSlavePort, "Receive response retry at %#x\n",
                              ev->pkt->getAddr());

        if (sendTimingResp(ev->pkt))
        {
            DPRINTF(TcuSlavePort, "Resume after successful retry at %#x\n",
                                  ev->pkt->getAddr());

            DPRINTF(TcuSlavePort, "Poping %p from queue\n", ev);
            pendingResponses.pop();
            delete ev;
        }
        else
            break;
    }
}

void
BaseTcu::TcuSlavePort::ResponseEvent::process()
{
    // if the XBar is busy, we can only send a response to a port once. this
    // is queued there and calls our recvRespRetry() if the XBar is idle again.
    // but we can't try another time. so, for the second time, don't even try
    // but directly queue it here
    if (port.pendingResponses.empty())
    {
        DPRINTF(TcuSlavePort, "Try to send %s response at %#x (%u bytes)\n",
                              pkt->cmd.toString(),
                              pkt->getAddr(),
                              pkt->getSize());

        if (!port.sendTimingResp(pkt))
        {
            DPRINTFS(TcuSlavePort, (&port), "Pushing %p to queue\n", this);
            port.pendingResponses.push(this);
        }
        // if it succeeded, let the event system delete the event
        else
            setFlags(AutoDelete);
    }
    else
    {
        DPRINTFS(TcuSlavePort, (&port), "Pushing %p to queue\n", this);
        port.pendingResponses.push(this);
    }
}

AddrRangeList
BaseTcu::NocSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;

    Addr baseNocAddr = NocAddr(tcu.coreId, 0).getAddr();
    Addr topNocAddr  = NocAddr(tcu.coreId + 1, 0).getAddr() - 1;

    DPRINTF(TcuSlavePort, "Tcu %u covers %#x to %#x\n",
                          tcu.coreId,
                          baseNocAddr,
                          topNocAddr);

    auto range = AddrRange(baseNocAddr, topNocAddr);

    ranges.push_back(range);

    return ranges;
}

AddrRangeList
BaseTcu::CacheMemSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, -1));
    return ranges;
}

BaseTcu::BaseTcu(BaseTcuParams* p)
  : ClockedObject(p),
    nocMasterPort(*this),
    nocSlavePort(*this),
    icacheMasterPort(*this),
    dcacheMasterPort(*this),
    icacheSlavePort(icacheMasterPort, *this, true),
    dcacheSlavePort(dcacheMasterPort, *this, false),
    cacheMemSlavePort(*this),
    caches(p->caches),
    nocReqFinishedEvent(*this),
    coreId(p->core_id),
    mmioRegion(p->mmio_region),
    slaveRegion(p->slave_region),
    coherent(p->coherent)
{
}

void
BaseTcu::init()
{
    ClockedObject::init();

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
BaseTcu::NocSlavePort::handleRequest(PacketPtr pkt,
                                     bool *busy,
                                     bool functional)
{
    *busy = true;

    tcu.handleNocRequest(pkt);

    return true;
}

bool
BaseTcu::CacheMemSlavePort::handleRequest(PacketPtr pkt,
                                          bool *busy,
                                          bool functional)
{
    // if that failed, it was an invalid request (probably due to speculative
    // execution)
    if (!tcu.handleCacheMemRequest(pkt, functional))
        tcu.sendDummyResponse(*this, pkt, functional);

    // in general, pretend that everything is fine
    return true;
}

void
BaseTcu::sendDummyResponse(TcuSlavePort &port, PacketPtr pkt, bool functional)
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
            DPRINTF(TcuSlavePort, "Sending dummy %s response at %#x (%u bytes) [senderState=%#x]\n",
                                  pkt->cmd.toString(),
                                  pkt->getAddr(),
                                  pkt->getSize(),
                                  pkt->senderState);

            // somehow we need to send that later to make the cache happy.
            port.schedTimingResp(pkt, clockEdge(Cycles(1)));
        }
    }
}

Port&
BaseTcu::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "icache_master_port")
        return icacheMasterPort;
    else if (if_name == "dcache_master_port")
        return dcacheMasterPort;
    else if (if_name == "noc_master_port")
        return nocMasterPort;
    else if (if_name == "icache_slave_port")
        return icacheSlavePort;
    else if (if_name == "dcache_slave_port")
        return dcacheSlavePort;
    else if (if_name == "noc_slave_port")
        return nocSlavePort;
    else if (if_name == "cache_mem_slave_port")
        return cacheMemSlavePort;
    else
        return SimObject::getPort(if_name, idx);
}

void
BaseTcu::schedNocResponse(PacketPtr pkt, Tick when)
{
    assert(pkt->isResponse());

    nocSlavePort.schedTimingResp(pkt, when);
}

void
BaseTcu::schedNocRequestFinished(Tick when)
{
    schedule(nocReqFinishedEvent, when);
}

void
BaseTcu::nocRequestFinished()
{
    nocSlavePort.requestFinished();
}

void
BaseTcu::schedCpuResponse(PacketPtr pkt, Tick when)
{
    assert(pkt->isResponse());

    dcacheSlavePort.schedTimingResp(pkt, when);
}

void
BaseTcu::sendCacheMemResponse(PacketPtr pkt, bool success)
{
    DPRINTF(TcuSlavePort, "Send %s response at %#x (%u bytes)\n",
            pkt->cmd.toString(),
            pkt->getAddr(),
            pkt->getSize());

    if (!success)
        sendDummyResponse(cacheMemSlavePort, pkt, false);
    else
        cacheMemSlavePort.schedTimingResp(pkt, clockEdge(Cycles(1)));
}

void
BaseTcu::schedNocRequest(PacketPtr pkt, Tick when)
{
    printNocRequest(pkt, "timing");
    nocMasterPort.schedTimingReq(pkt, when);
}

void
BaseTcu::schedMemRequest(PacketPtr pkt, Tick when)
{
    dcacheMasterPort.schedTimingReq(pkt, when);
}

void
BaseTcu::sendFunctionalNocRequest(PacketPtr pkt)
{
    printNocRequest(pkt, "functional");
    nocMasterPort.sendFunctional(pkt);
}

void
BaseTcu::sendAtomicNocRequest(PacketPtr pkt)
{
    printNocRequest(pkt, "atomic");
    nocMasterPort.sendAtomic(pkt);
}

void
BaseTcu::printNocRequest(PacketPtr pkt, const char *type)
{
    DPRINTFS(TcuMasterPort,
             (&nocMasterPort),
             "Sending %s %s at %#x (%u bytes)\n",
             type,
             pkt->cmd.toString(),
             pkt->getAddr(),
             pkt->getSize());
}

void
BaseTcu::sendAtomicMemRequest(PacketPtr pkt)
{
    dcacheMasterPort.sendAtomic(pkt);
}
