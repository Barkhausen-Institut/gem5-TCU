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

#include <iomanip>
#include <sstream>

#include "debug/Tcu.hh"
#include "debug/TcuPackets.hh"
#include "debug/TcuMem.hh"
#include "debug/TcuCoreMemAcc.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/msg_unit.hh"
#include "mem/tcu/mem_unit.hh"
#include "mem/tcu/xfer_unit.hh"
#include "mem/cache/cache.hh"
#include "sim/tile_memory.hh"
#include "sim/system.hh"

Tcu::Tcu(const TcuParams &p)
  : BaseTcu(p),
    requestorId(p.system->getRequestorId(this, name())),
    system(p.system),
    regFile(*this, name() + ".regFile", p.num_endpoints),
    connector(p.connector),
    tlBuf(p.tlb_entries > 0 ? new TcuTlb(*this, p.tlb_entries) : NULL),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, p.block_size, p.buf_count, p.buf_size)),
    coreReqs(*this, p.buf_count),
    epFile(*this),
    sleepEPs(epFile.newCache()),
    cmds(*this),
    fireTimerEvent(*this),
    completeCoreReqEvent(coreReqs),
    wakeupEp(0xFFFF),
    tileMemOffset(p.tile_mem_offset),
    atomicMode(p.system->isAtomicMode()),
    numEndpoints(p.num_endpoints),
    maxNocPacketSize(p.max_noc_packet_size),
    blockSize(p.block_size),
    bufCount(p.buf_count),
    bufSize(p.buf_size),
    reqCount(p.req_count),
    cacheBlocksPerCycle(p.cache_blocks_per_cycle),
    registerAccessLatency(p.register_access_latency),
    cpuToCacheLatency(p.cpu_to_cache_latency),
    commandToNocRequestLatency(p.command_to_noc_request_latency),
    startMsgTransferDelay(p.start_msg_transfer_delay),
    transferToMemRequestLatency(p.transfer_to_mem_request_latency),
    transferToNocLatency(p.transfer_to_noc_latency),
    nocToTransferLatency(p.noc_to_transfer_latency)
{
    assert(p.buf_size >= maxNocPacketSize);

    connector->setTcu(this);
}

Tcu::~Tcu()
{
    delete xferUnit;
    delete memUnit;
    delete msgUnit;
    delete tlBuf;
}

void
Tcu::regStats()
{
    BaseTcu::regStats();

    nocMsgRecvs
        .name(name() + ".nocMsgRecvs")
        .desc("Number of received messages");
    nocReadRecvs
        .name(name() + ".nocReadRecvs")
        .desc("Number of received read requests");
    nocWriteRecvs
        .name(name() + ".nocWriteRecvs")
        .desc("Number of received write requests");

    regFileReqs
        .name(name() + ".regFileReqs")
        .desc("Number of requests to the register file");
    intMemReqs
        .name(name() + ".intMemReqs")
        .desc("Number of requests to the internal memory");
    extMemReqs
        .name(name() + ".extMemReqs")
        .desc("Number of requests to the external memory");
    irqInjects
        .name(name() + ".irqInjects")
        .desc("Number of injected IRQs");
    resets
        .name(name() + ".resets")
        .desc("Number of resets");

    if (tlb())
        tlb()->regStats();
    cmds.regStats();
    coreReqs.regStats();
    xferUnit->regStats();
    memUnit->regStats();
    msgUnit->regStats();
}

bool
Tcu::isMemTile(unsigned tile) const
{
    TileMemory *sys = dynamic_cast<TileMemory*>(system);
    return !sys || sys->hasMem(tile);
}

PacketPtr
Tcu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
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
Tcu::freeRequest(PacketPtr pkt)
{
    delete pkt;
}

void
Tcu::startWaitEP(const CmdCommand::Bits &cmd)
{
    int ep = cmd.arg0 & 0xFFFF;

    if (ep == INVALID_EP_ID)
    {
        if (regs().getAct(PrivReg::CUR_ACT).msgs > 0)
        {
            scheduleCmdFinish(Cycles(1), TcuError::NONE);
            return;
        }

        if (!startSleep(ep))
            scheduleCmdFinish(Cycles(1), TcuError::NONE);
    }
    else
    {
        sleepEPs.addEp(ep);
        sleepEPs.onFetched(std::bind(&Tcu::startWaitEPWithEP,
                                     this, std::placeholders::_1, ep));
    }
}

void
Tcu::startWaitEPWithEP(EpFile::EpCache &eps, epid_t epid)
{
    const Ep ep = eps.getEp(epid);
    if (ep.type() == EpType::RECEIVE && ep.recv.r2.unread != 0)
    {
        scheduleCmdFinish(Cycles(1), TcuError::NONE);
        return;
    }

    if (!startSleep(epid))
        scheduleCmdFinish(Cycles(1), TcuError::NONE);
}

bool
Tcu::startSleep(epid_t ep)
{
    if (connector->havePendingIrq())
        return false;

    wakeupEp = ep;
    DPRINTF(Tcu, "Suspending CU (waiting for EP %d)\n", wakeupEp);
    connector->suspend();

    return true;
}

void
Tcu::stopSleep()
{
    connector->wakeup();
}

void
Tcu::wakeupCore(bool force, epid_t rep)
{
    if (force || wakeupEp == INVALID_EP_ID || rep == wakeupEp)
    {
        // better stop the command in this cycle to ensure that the core
        // does not issue another command before we can finish the sleep.
        if (regs().getCommand().opcode == CmdCommand::SLEEP)
            scheduleCmdFinish(Cycles(0));
        else
            connector->wakeup();
    }
}

Cycles
Tcu::reset(bool flushInval)
{
    Cycles delay = flushInval ? flushInvalCaches(true) : Cycles(0);

    if (tlb())
        tlb()->clear();

    connector->reset();

    resets++;
    return delay;
}

Cycles
Tcu::flushInvalCaches(bool invalidate)
{
    Cycles delay(0);
    for (auto &c : caches)
    {
        c->memWriteback();
        if (invalidate)
            c->memInvalidate();
        delay += Cycles(c->getBlockCount() / cacheBlocksPerCycle);
    }
    return delay;
}

void
Tcu::setIrq(BaseConnector::IRQ irq)
{
    wakeupCore(true, INVALID_EP_ID);

    connector->setIrq(irq);

    irqInjects++;
}

void
Tcu::clearIrq(BaseConnector::IRQ irq)
{
    connector->clearIrq(irq);
}

void
Tcu::fireTimer()
{
    setIrq(BaseConnector::IRQ::TIMER);
}

void
Tcu::restartTimer(uint64_t nanos)
{
    if (fireTimerEvent.scheduled())
        deschedule(&fireTimerEvent);
    if (nanos != 0)
    {
        Cycles sleep_time = ticksToCycles(nanos * 1000);
        schedule(&fireTimerEvent, clockEdge(sleep_time));
    }
}

void
Tcu::printLine(Addr len)
{
    const char *buffer = regs().getBuffer(len);
    DPRINTF(Tcu, "PRINT: %s\n", buffer);
}

void
Tcu::sendMemRequest(PacketPtr pkt,
                    Addr data,
                    Cycles delay)
{
    auto senderState = new MemSenderState();
    senderState->data = data;
    senderState->mid = pkt->req->requestorId();

    // ensure that this packet has our master id (not the id of a master in
    // a different tile)
    pkt->req->setRequestorId(requestorId);

    pkt->pushSenderState(senderState);

    if (atomicMode)
    {
        sendAtomicMemRequest(pkt);
        completeMemRequest(pkt);
    }
    else
    {
        schedMemRequest(pkt, clockEdge(delay));
    }
}

void
Tcu::sendNocRequest(NocPacketType type,
                    PacketPtr pkt,
                    Cycles delay,
                    bool functional)
{
    auto senderState = new NocSenderState();
    senderState->packetType = type;
    senderState->result = TcuError::NONE;

    if (type == NocPacketType::MESSAGE || type == NocPacketType::READ_REQ ||
        type == NocPacketType::WRITE_REQ)
        cmds.setRemoteCommand(true);

    pkt->pushSenderState(senderState);

    if (functional)
    {
        sendFunctionalNocRequest(pkt);
        completeNocRequest(pkt);
    }
    else if (atomicMode)
    {
        sendAtomicNocRequest(pkt);
        completeNocRequest(pkt);
    }
    else
    {
        schedNocRequest(pkt, clockEdge(delay));
    }
}

void
Tcu::sendNocResponse(PacketPtr pkt, TcuError result)
{
    auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);
    senderState->result = result;

    pkt->makeResponse();

    if (!atomicMode)
    {
        Cycles delay = ticksToCycles(
            pkt->headerDelay + pkt->payloadDelay);
        delay += nocToTransferLatency;

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        schedNocRequestFinished(clockEdge(Cycles(1)));
        schedNocResponse(pkt, clockEdge(delay));
    }
}

void
Tcu::startTransfer(void *event, Cycles delay)
{
    auto ev = reinterpret_cast<XferUnit::TransferEvent*>(event);
    xferUnit->startTransfer(ev, delay);
}

size_t
Tcu::startForeignReceive(epid_t epId, actid_t actId)
{
    // if a command is running, send the response now to finish its memory
    // write instruction to the COMMAND register
    cmds.stopCommand();

    return coreReqs.startForeignReceive(epId, actId);
}

void
Tcu::completeNocRequest(PacketPtr pkt)
{
    auto senderState = dynamic_cast<NocSenderState*>(pkt->popSenderState());

    if (senderState->packetType == NocPacketType::CACHE_MEM_REQ)
    {
        // as these target memory tiles, there can't be any error
        assert(senderState->result == TcuError::NONE);

        if (auto state = dynamic_cast<InitSenderState*>(pkt->senderState))
        {
            // undo the change from handleCacheMemRequest
            pkt->setAddr(state->oldAddr);
            pkt->req->setPaddr(state->oldAddr);
            pkt->popSenderState();
        }

        DPRINTF(TcuMem,
            "Finished %s request of LLC for %u bytes @ %#x\n",
            pkt->isRead() ? "read" : "write",
            pkt->getSize(), pkt->getAddr());

        if(pkt->isRead())
            printPacket(pkt);

        sendCacheMemResponse(pkt, true);
    }
    else if (senderState->packetType != NocPacketType::CACHE_MEM_REQ_FUNC)
    {
        cmds.setRemoteCommand(false);
        // if the current command should be aborted, just ignore the packet
        // and finish the command
        if (cmds.isCommandAborting())
            scheduleCmdFinish(Cycles(1), TcuError::ABORT);
        else
        {
            auto cmd = regs().getCommand();
            if (pkt->isWrite())
                memUnit->writeComplete(cmd, pkt, senderState->result);
            else if (pkt->isRead())
                memUnit->readComplete(cmd, pkt, senderState->result);
            else
                panic("unexpected packet type\n");
        }
    }

    delete senderState;
}

void
Tcu::completeMemRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isResponse());

    auto senderState = dynamic_cast<MemSenderState*>(pkt->popSenderState());

    // set the old requestor id again
    pkt->req->setRequestorId(senderState->mid);

    xferUnit->recvMemResponse(senderState->data, pkt);

    delete senderState;
    freeRequest(pkt);
}

void
Tcu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());

    if (pkt->cacheResponding())
    {
        DPRINTF(TcuPackets, "Ignoring packet, because cache is responding\n");
        schedNocRequestFinished(clockEdge(Cycles(1)));
        return;
    }

    auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);

    switch (senderState->packetType)
    {
        case NocPacketType::MESSAGE:
        {
            nocMsgRecvs++;
            msgUnit->recvFromNoc(pkt);
            break;
        }
        case NocPacketType::READ_REQ:
        case NocPacketType::WRITE_REQ:
        case NocPacketType::CACHE_MEM_REQ:
        {
            if (senderState->packetType == NocPacketType::READ_REQ)
                nocReadRecvs++;
            else if (senderState->packetType == NocPacketType::WRITE_REQ)
                nocWriteRecvs++;
            memUnit->recvFromNoc(pkt);
            break;
        }
        case NocPacketType::CACHE_MEM_REQ_FUNC:
            memUnit->recvFunctionalFromNoc(pkt);
            break;
        default:
            panic("Unexpected NocPacketType\n");
    }
}

bool
Tcu::handleCoreMemRequest(PacketPtr pkt,
                          TcuSlavePort &sport,
                          TcuMasterPort &mport,
                          bool icache,
                          bool functional)
{
    bool res = true;
    Addr virt = pkt->getAddr();

    DPRINTF(TcuCoreMemAcc, "%s access for %#lx: start\n",
        pkt->cmdString(), virt);

    if (mmioRegion.contains(virt))
    {
        // not supported here
        assert(!functional);

        if (icache)
            res = false;
        else
            forwardRequestToRegFile(pkt, true);
    }
    else
    {
        intMemReqs++;

        if (functional)
            mport.sendFunctional(pkt);
        else
        {
            Tick tick;
            if (cpuToCacheLatency < Cycles(1))
                tick = curTick();
            else
                tick = clockEdge(cpuToCacheLatency);
            mport.schedTimingReq(pkt, tick);
        }
    }

    DPRINTF(TcuCoreMemAcc, "%s access for %#lx: finished\n",
        pkt->cmdString(), virt);

    return res;
}

NocAddr
Tcu::translatePhysToNoC(Addr phys, bool write)
{
    Addr physAddr = phys - tileMemOffset;
    Addr physOff = physAddr & 0x3FFFFFFF;
    epid_t epid = physAddr >> 30;

    if (epid >= numEndpoints || regs().getEp(epid).type() != EpType::MEMORY)
    {
        DPRINTFS(Tcu, this, "PMP-EP%u: invalid EP (phys=%#x)\n", epid, phys);
        warn("T%u,PMP-EP%u: invalid EP", tileId, epid);
        return NocAddr();
    }

    const MemEp mep = regs().getEp(epid).mem;

    if (physOff >= mep.r2.remoteSize)
    {
        DPRINTFS(Tcu, this,
                 "PMP-EP%u: out of bounds (%#x vs. %#x)\n",
                 epid, physOff, mep.r2.remoteSize);
        warn("T%u,PMP-EP%u: out of bounds", tileId, epid);
        return NocAddr();
    }
    if ((!write && !(mep.r0.flags & MemoryFlags::READ)) ||
        (write && !(mep.r0.flags & MemoryFlags::WRITE)))
    {
        DPRINTFS(Tcu, this,
                 "PMP-EP%u: permission denied (flags=%#x, write=%d)\n",
                 epid, mep.r0.flags, write);
        warn("T%u,PMP-EP%u: permission denied", tileId, epid);
        return NocAddr();
    }

    // translate to NoC address
    return NocAddr(mep.r0.targetTile, mep.r1.remoteAddr + physOff);
}

bool
Tcu::handleCacheMemRequest(PacketPtr pkt, bool functional)
{
    if (pkt->cmd == MemCmd::CleanEvict)
    {
        assert(!pkt->needsResponse());
        DPRINTF(TcuPackets, "Dropping CleanEvict packet\n");
        return true;
    }

    // we don't have cache coherence. so we don't care about invalidate req.
    if (pkt->cmd == MemCmd::InvalidateReq)
        return false;
    if (pkt->cmd == MemCmd::BadAddressError)
        return false;

    Addr pktAddr = pkt->getAddr();
    // assert((pktAddr & (pkt->getSize() - 1)) == 0);
    NocAddr noc = translatePhysToNoC(pktAddr, pkt->isWrite());
    if (!noc.valid)
        return false;

    pkt->setAddr(noc.getAddr());

    DPRINTF(TcuMem, "Sending LLC request for %#x to %d:%#x\n",
                    pktAddr, noc.tileId, noc.offset);

    if(pkt->isWrite())
        printPacket(pkt);

    // remember that we did this change
    if (!functional)
        pkt->pushSenderState(new InitSenderState(pktAddr));

    auto type = functional ? Tcu::NocPacketType::CACHE_MEM_REQ_FUNC
                           : Tcu::NocPacketType::CACHE_MEM_REQ;
    // this does always target a memory tile, so actId is invalid
    sendNocRequest(type,
                   pkt,
                   Cycles(1),
                   functional);

    extMemReqs++;

    if (functional)
        pkt->setAddr(pktAddr);
    return true;
}

void
Tcu::forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest)
{
    Addr oldAddr = pkt->getAddr();

    // Strip the base address to handle requests based on the reg. addr. only.
    pkt->setAddr(oldAddr - mmioRegion.start());

    RegFile::Result result = regs().handleRequest(pkt, isCpuRequest);

    regFileReqs++;

    // restore old address
    pkt->setAddr(oldAddr);

    /*
     * We handle the request immediatly and do not care about timing. The
     * delay is payed by scheduling the response at some point in the
     * future. Additionaly a write operation on the command register needs
     * to schedule an event that executes this command at a future tick.
     */

    Cycles transportDelay =
        ticksToCycles(pkt->headerDelay + pkt->payloadDelay);

    Tick when = clockEdge(transportDelay + registerAccessLatency);

    if (!isCpuRequest)
        schedNocRequestFinished(clockEdge(Cycles(1)));

    if (~result & RegFile::WROTE_EXT_CMD)
    {
        assert(isCpuRequest || result == RegFile::WROTE_NONE);
        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        if (isCpuRequest &&
            (result & (RegFile::WROTE_CMD | RegFile::WROTE_PRIV_CMD)) == 0)
            schedCpuResponse(pkt, when);
        else if(!isCpuRequest)
            schedNocResponse(pkt, when);

        if (result & RegFile::WROTE_CORE_REQ)
            schedule(completeCoreReqEvent, when);
        if (result & RegFile::WROTE_PRINT)
        {
            printLine(regs().get(UnprivReg::PRINT));
            regs().set(UnprivReg::PRINT, 0);
        }
        if (result & RegFile::WROTE_CLEAR_IRQ)
            clearIrq((BaseConnector::IRQ)regs().get(PrivReg::CLEAR_IRQ));
    }

    cmds.startCommand(result, pkt, when);
}

void
Tcu::sendFunctionalMemRequest(PacketPtr pkt)
{
    // set our master id (it might be from a different tile)
    pkt->req->setRequestorId(requestorId);

    dcacheMasterPort.sendFunctional(pkt);
}

void
Tcu::printPacket(PacketPtr pkt) const
{
    DPRINTF(TcuPackets, "Dumping packet %s @ %p with %lu bytes\n",
        pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    DDUMP(TcuPackets, pkt->getPtr<uint8_t>(), pkt->getSize());
}
