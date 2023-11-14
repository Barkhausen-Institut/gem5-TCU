/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
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

#include "debug/TcuResponsePort.hh"
#include "debug/TcuRequestPort.hh"
#include "debug/Tcu.hh"
#include "base/trace.hh"
#include "mem/tcu/base.hh"
#include "mem/tcu/noc_addr.hh"
#include "sim/system.hh"

namespace gem5
{
namespace tcu
{

BaseTcu::TcuRequestPort::TcuRequestPort(const std::string& _name, BaseTcu& _tcu)
  : QueuedRequestPort(_name, reqQueue, snoopRespQueue),
    tcu(_tcu),
    reqQueue(_tcu, *this),
    snoopRespQueue(_tcu, *this)
{ }

bool
BaseTcu::TcuRequestPort::recvTimingResp(PacketPtr pkt)
{
    // If we receive a response we should always expect it
    completeRequest(pkt);

    return true;
}

void
BaseTcu::NocRequestPort::completeRequest(PacketPtr pkt)
{
    DPRINTF(TcuRequestPort,
            "Received %s at %#x (%u bytes)\n",
            pkt->cmd.toString(),
            pkt->getAddr(),
            pkt->getSize());

    tcu.completeNocRequest(pkt);
}

bool
BaseTcu::ICacheRequestPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(TcuResponsePort, "Sending timing response at %#x [senderState=%#x]\n",
                          pkt->getAddr(),
                          pkt->senderState);

    // the TCU does never send requests to the icache. so just pass it back to
    // the CPU
    tcu.icacheResponsePort.schedTimingResp(pkt, tcu.clockEdge(Cycles(1)));
    return true;
}

bool
BaseTcu::DCacheRequestPort::recvTimingResp(PacketPtr pkt)
{
    // if there is a context-id and thread-id, the request came from the CPU
    if (pkt->req->hasContextId())
    {
        DPRINTF(TcuResponsePort, "Sending timing response at %#x [senderState=%#x]\n",
                              pkt->getAddr(),
                              pkt->senderState);

        tcu.dcacheResponsePort.schedTimingResp(pkt, tcu.clockEdge(Cycles(1)));
        return true;
    }

    // otherwise from the TCU
    tcu.completeMemRequest(pkt);
    return true;
}

BaseTcu::TcuResponsePort::TcuResponsePort(const std::string& _name, BaseTcu& _tcu)
  : ResponsePort(_name),
    tcu(_tcu),
    busy(false),
    sendReqRetry(false),
    pendingResponses()
{ }

void
BaseTcu::TcuResponsePort::requestFinished()
{
    assert(busy);
    busy = false;

    DPRINTF(TcuResponsePort, "Timing request finished\n");

    if (sendReqRetry)
    {
        DPRINTF(TcuResponsePort, "Send request retry\n");

        sendReqRetry = false;
        sendRetryReq();
    }
}

void
BaseTcu::TcuResponsePort::schedTimingResp(PacketPtr pkt, Tick when)
{
    DPRINTF(TcuResponsePort, "Schedule timing response %#x at Tick %u\n",
                          pkt->getAddr(),
                          when);

    assert(pkt->isResponse());

    auto respEvent = new ResponseEvent(*this, pkt);
    tcu.schedule(respEvent, when);
}

Tick
BaseTcu::TcuResponsePort::recvAtomic(PacketPtr pkt)
{
    panic("Atomic mode is not supported by the TCU!");
    return 0;
}

void
BaseTcu::TcuResponsePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(TcuResponsePort, "Receive functional %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

    // don't actually make us busy/unbusy here, because we might interfere
    // with the timing requests
    bool dummy = false;
    handleRequest(pkt, &dummy, true);
}

bool
BaseTcu::TcuResponsePort::recvTimingReq(PacketPtr pkt)
{
    if (busy)
    {
        DPRINTF(TcuResponsePort, "Reject timing %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

        sendReqRetry = true;
        return false;
    }

    DPRINTF(TcuResponsePort, "Receive timing %s request at %#x (%u bytes)\n",
                          pkt->cmd.toString(),
                          pkt->getAddr(),
                          pkt->getSize());

    assert(!sendReqRetry);

    return handleRequest(pkt, &busy, false);
}

void
BaseTcu::TcuResponsePort::recvRespRetry()
{
    // try to send all queued responses. the first one should always succeed
    // because the XBar called us because it is free. the second should always
    // fail since it is busy then. in this case, we stop here.
    while (!pendingResponses.empty())
    {
        ResponseEvent *ev = pendingResponses.front();

        DPRINTF(TcuResponsePort, "Receive response retry at %#x\n",
                              ev->pkt->getAddr());

        if (sendTimingResp(ev->pkt))
        {
            DPRINTF(TcuResponsePort, "Resume after successful retry at %#x\n",
                                  ev->pkt->getAddr());

            DPRINTF(TcuResponsePort, "Poping %p from queue\n", ev);
            pendingResponses.pop();
            delete ev;
        }
        else
            break;
    }
}

void
BaseTcu::TcuResponsePort::ResponseEvent::process()
{
    // if the XBar is busy, we can only send a response to a port once. this
    // is queued there and calls our recvRespRetry() if the XBar is idle again.
    // but we can't try another time. so, for the second time, don't even try
    // but directly queue it here
    if (port.pendingResponses.empty())
    {
        DPRINTF(TcuResponsePort, "Try to send %s response at %#x (%u bytes)\n",
                              pkt->cmd.toString(),
                              pkt->getAddr(),
                              pkt->getSize());

        if (!port.sendTimingResp(pkt))
        {
            DPRINTFS(TcuResponsePort, (&port), "Pushing %p to queue\n", this);
            port.pendingResponses.push(this);
        }
        // if it succeeded, let the event system delete the event
        else
            setFlags(AutoDelete);
    }
    else
    {
        DPRINTFS(TcuResponsePort, (&port), "Pushing %p to queue\n", this);
        port.pendingResponses.push(this);
    }
}

AddrRangeList
BaseTcu::NocResponsePort::getAddrRanges() const
{
    AddrRangeList ranges;

    Addr baseNocAddr = NocAddr(tcu.tileId, 0).getAddr();
    Addr topNocAddr  = NocAddr(TileId::from_raw(tcu.tileId.raw() + 1), 0).getAddr() - 1;

    DPRINTF(TcuResponsePort, "Covering %#x to %#x\n", baseNocAddr, topNocAddr);

    auto range = AddrRange(baseNocAddr, topNocAddr);

    ranges.push_back(range);

    return ranges;
}

bool
BaseTcu::NocResponsePort::handleRequest(PacketPtr pkt,
                                     bool *busy,
                                     bool functional)
{
    *busy = true;

    tcu.handleNocRequest(pkt);

    return true;
}

AddrRangeList
BaseTcu::LLCResponsePort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, -1));
    return ranges;
}

bool
BaseTcu::LLCResponsePort::handleRequest(PacketPtr pkt,
                                          bool *busy,
                                          bool functional)
{
    // if that failed, it was an invalid request (probably due to speculative
    // execution)
    if (!tcu.handleLLCRequest(pkt, functional))
        tcu.schedDummyResponse(*this, pkt, functional);

    // in general, pretend that everything is fine
    return true;
}

BaseTcu::BaseTcu(const BaseTcuParams &p)
  : ClockedObject(p),
    system(p.system),
    requestorId(p.system->getRequestorId(this, name())),
    nocRequestPort(*this),
    nocResponsePort(*this),
    icacheRequestPort(*this),
    dcacheRequestPort(*this),
    icacheResponsePort(icacheRequestPort, *this, true),
    dcacheResponsePort(dcacheRequestPort, *this, false),
    llcResponsePort(*this),
    nocReqFinishedEvent(*this),
    caches(p.caches),
    tileId(TileId::from_raw(p.tile_id)),
    mmioRegion(p.mmio_region),
    slaveRegion(p.slave_region)
{
}

void
BaseTcu::init()
{
    ClockedObject::init();

    assert(nocRequestPort.isConnected());
    assert(nocResponsePort.isConnected());

    nocResponsePort.sendRangeChange();

    // for memory tiles, the icache/dcache slaves are not connected
    if (icacheResponsePort.isConnected())
        icacheResponsePort.sendRangeChange();
    if (dcacheResponsePort.isConnected())
        dcacheResponsePort.sendRangeChange();

    // the cache-mem slave port is only used if we have a cache
    if (llcResponsePort.isConnected())
        llcResponsePort.sendRangeChange();
}

Port&
BaseTcu::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "icache_master_port")
        return icacheRequestPort;
    else if (if_name == "dcache_master_port")
        return dcacheRequestPort;
    else if (if_name == "noc_master_port")
        return nocRequestPort;
    else if (if_name == "icache_slave_port")
        return icacheResponsePort;
    else if (if_name == "dcache_slave_port")
        return dcacheResponsePort;
    else if (if_name == "noc_slave_port")
        return nocResponsePort;
    else if (if_name == "llc_slave_port")
        return llcResponsePort;
    else
        return SimObject::getPort(if_name, idx);
}

// -- requests --

PacketPtr
BaseTcu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
{
    Request::Flags flags;

    auto req = std::make_shared<Request>(paddr, size, flags, requestorId);

    auto pkt = new Packet(req, cmd);

    if (size)
    {
        auto pktData = new uint8_t[size];
        pkt->dataDynamic(pktData);
    }

    return pkt;
}

void
BaseTcu::freeRequest(PacketPtr pkt)
{
    delete pkt;
}

void
BaseTcu::schedNocRequest(PacketPtr pkt, Tick when)
{
    printNocRequest(pkt, "timing");
    nocRequestPort.schedTimingReq(pkt, when);
}

void
BaseTcu::schedMemRequest(PacketPtr pkt, Tick when)
{
    // ensure that this packet has our master id (not the id of a master in
    // a different tile)
    pkt->req->setRequestorId(requestorId);

    dcacheRequestPort.schedTimingReq(pkt, when);
}

void
BaseTcu::sendFunctionalNocRequest(PacketPtr pkt)
{
    printNocRequest(pkt, "functional");
    nocRequestPort.sendFunctional(pkt);
}

void
BaseTcu::sendFunctionalMemRequest(PacketPtr pkt)
{
    // set our master id (it might be from a different tile)
    pkt->req->setRequestorId(requestorId);

    dcacheRequestPort.sendFunctional(pkt);
}

// -- responses --

void
BaseTcu::schedNocResponse(PacketPtr pkt, Tick when)
{
    assert(pkt->isResponse());

    nocResponsePort.schedTimingResp(pkt, when);
}

void
BaseTcu::schedCpuResponse(PacketPtr pkt, Tick when)
{
    assert(pkt->isResponse());

    dcacheResponsePort.schedTimingResp(pkt, when);
}

void
BaseTcu::schedLLCResponse(PacketPtr pkt, bool success)
{
    DPRINTF(TcuResponsePort, "Send %s response at %#x (%u bytes)\n",
            pkt->cmd.toString(),
            pkt->getAddr(),
            pkt->getSize());

    if (!success)
        schedDummyResponse(llcResponsePort, pkt, false);
    else
        llcResponsePort.schedTimingResp(pkt, clockEdge(Cycles(1)));
}

// -- misc --

void
BaseTcu::invalidateCaches()
{
    for(auto cache : caches)
    {
        cache->memWriteback();
        cache->memInvalidate();
    }
}

void
BaseTcu::schedNocRequestFinished(Tick when)
{
    schedule(nocReqFinishedEvent, when);
}

void
BaseTcu::nocRequestFinished()
{
    nocResponsePort.requestFinished();
}

void
BaseTcu::schedDummyResponse(TcuResponsePort &port, PacketPtr pkt, bool functional)
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
            DPRINTF(TcuResponsePort, "Sending dummy %s response at %#x (%u bytes) [senderState=%#x]\n",
                                  pkt->cmd.toString(),
                                  pkt->getAddr(),
                                  pkt->getSize(),
                                  pkt->senderState);

            // somehow we need to send that later to make the cache happy.
            port.schedTimingResp(pkt, clockEdge(Cycles(1)));
        }
    }
}

void
BaseTcu::printNocRequest(PacketPtr pkt, const char *type)
{
    DPRINTFS(TcuRequestPort,
             (&nocRequestPort),
             "Sending %s %s at %#x (%u bytes)\n",
             type,
             pkt->cmd.toString(),
             pkt->getAddr(),
             pkt->getSize());
}

}
}
