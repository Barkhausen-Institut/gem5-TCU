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
#include <ostream>

#include "arch/generic/mmu.hh"
#include "base/output.hh"
#include "cpu/base.hh"
#include "debug/Tcu.hh"
#include "debug/TcuPackets.hh"
#include "debug/TcuLLCMemAcc.hh"
#include "debug/TcuCoreMemAcc.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/msg_unit.hh"
#include "mem/tcu/mem_unit.hh"
#include "mem/tcu/xfer_unit.hh"
#include "mem/cache/cache.hh"
#include "sim/tile_memory.hh"
#include "sim/system.hh"

namespace gem5
{
namespace tcu
{

Tcu::Tcu(const TcuParams &p)
  : BaseTcu(p),
    regFile(*this, name() + ".regFile", p.num_endpoints),
    connector(*this, p.connector),
    tlBuf(p.tlb_entries > 0 ? new TcuTlb(*this, p.tlb_entries) : NULL),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, p.block_size, p.buf_count, p.buf_size)),
    cuReqs(*this, p.buf_count),
    epFile(*this),
    cmds(*this),
    completeCUReqEvent(cuReqs),
    coreDrained(),
    tileMemOffset(p.tile_mem_offset),
    numEndpoints(p.num_endpoints),
    maxNocPacketSize(p.max_noc_packet_size),
    blockSize(p.block_size),
    bufCount(p.buf_count),
    bufSize(p.buf_size),
    reqCount(p.req_count),
    mmioLatency(p.mmio_latency),
    cpuToCacheLatency(p.cpu_to_cache_latency),
    tlbLatency(p.tlb_latency),
    cmdReadLatency(p.cmd_read_latency),
    cmdWriteLatency(p.cmd_write_latency),
    cmdSendLatency(p.cmd_send_latency),
    cmdReplyLatency(p.cmd_reply_latency),
    cmdRecvLatency(p.cmd_recv_latency),
    cmdFetchLatency(p.cmd_fetch_latency),
    cmdAckLatency(p.cmd_ack_latency)
{
    assert(p.buf_size >= maxNocPacketSize);
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
    resets
        .name(name() + ".resets")
        .desc("Number of resets");

    if (tlb())
        tlb()->regStats();
    cmds.regStats();
    cuReqs.regStats();
    xferUnit->regStats();
    memUnit->regStats();
    msgUnit->regStats();
    connector.regStats();
}

const std::string
Tcu::ResetEvent::name() const
{
    return _tcu.name();
}

void
Tcu::ResetEvent::process()
{
    if (_tcu.system->threads.empty() ||
        _tcu.system->threads[0]->getCpuPtr()->drain() == DrainState::Drained)
    {
        _tcu.connector.startSleep(Tcu::INVALID_EP_ID, true);
        _tcu.invalidateCaches();
        _tcu.regs().reset(true);

        _tcu.coreDrained = !_tcu.system->threads.empty();
        _tcu.connector.reset(false);
        _tcu.scheduleExtCmdFinish(Cycles(1), TcuError::NONE, 0);
        delete this;
    }
    else
        _tcu.schedule(this, _tcu.clockEdge(Cycles(1)));
}

void
Tcu::reset(bool start)
{
    resets++;

    if (tlb())
        tlb()->clear();

    if (start)
    {
        if (coreDrained)
        {
            system->threads[0]->getCpuPtr()->drainResume();
            coreDrained = false;
        }
        connector.reset(true);
        connector.stopSleep();
        scheduleExtCmdFinish(Cycles(1), TcuError::NONE, 0);
    }
    else
    {
        auto resetEvent = new ResetEvent(*this);
        schedule(resetEvent, clockEdge(Cycles(1)));
    }
}

void
Tcu::printLine(Addr len)
{
    const char *buffer = regs().getBuffer(len);
    DPRINTF(Tcu, "PRINT: %s\n", buffer);
    regs().set(UnprivReg::PRINT, 0);
}

void
Tcu::writeCoverage(PrintReg pr)
{
    if (pr.size == 0)
        regs().set(UnprivReg::PRINT, 0);
    else
    {
        auto writeCovEvent = new WriteCoverageEvent(*this, pr.cov_act,
                                                    pr.cov_addr, pr.size);
        schedule(writeCovEvent, clockEdge(Cycles(1)));
    }
}

const std::string
Tcu::WriteCoverageEvent::name() const
{
    return _tcu.name();
}

void
Tcu::WriteCoverageEvent::process()
{
    Request::Flags flags;

    auto req = std::make_shared<Request>(_gen.addr(), _gen.size(),
                                         flags, _tcu.requestorId);
    req->setVirt(_gen.addr(), _gen.size(), flags, _tcu.requestorId, 0);

    BaseMMU::Mode mode = BaseMMU::Read;
    WholeTranslationState *state =
        new WholeTranslationState(req, new uint8_t[_gen.size()], NULL, mode);
    DataTranslation<Tcu::WriteCoverageEvent *> *translation
        = new DataTranslation<Tcu::WriteCoverageEvent *>(this, state);

    auto tc = _tcu.system->threads[0];
    tc->getMMUPtr()->translateTiming(req, tc, translation, mode);
}

void
Tcu::WriteCoverageEvent::finishTranslation(WholeTranslationState *state)
{
    assert(state->getFault() == NoFault);
    assert(state->isSplit == false);

    auto pkt = new Packet(state->mainReq, MemCmd::ReadReq);
    pkt->dataStatic(state->data);
    _tcu.sendMemRequest(pkt, reinterpret_cast<Addr>(this), Cycles(1), true);

    delete state;
}

void
Tcu::WriteCoverageEvent::completed(PacketPtr pkt)
{
    if (!_out)
    {
        std::ostringstream filename;
        filename << "coverage-" << _tcu.tileId.raw() << "-" << _act << ".profraw";

        DPRINTF(Tcu, "Writing coverage: opening %s\n", filename.str().c_str());

        _out = simout.open(filename.str(),
                           std::ios::app | std::ios::out | std::ios::binary,
                           false, true);
        _os = _out->stream();
        if (!_os)
            panic("could not open file %s\n", filename.str().c_str());
    }

    DPRINTF(Tcu, "Writing coverage: %d bytes from %#x:%#x\n",
        pkt->getSize(), _gen.addr(), pkt->getAddr());

    _os->write(pkt->getPtr<char>(), pkt->getSize());

    _gen.next();
    if(_gen.done())
    {
        _tcu.regs().set(UnprivReg::PRINT, 0);
        delete this;
    }
    else
        _tcu.schedule(this, _tcu.clockEdge(Cycles(1)));
}

void
Tcu::startTransfer(void *event, Cycles delay)
{
    auto ev = reinterpret_cast<XferUnit::TransferEvent*>(event);
    xferUnit->startTransfer(ev, delay);
}

void
Tcu::startForeignReceive(epid_t epId, actid_t actId)
{
    // if a command is running, send the response now to finish its memory
    // write instruction to the COMMAND register
    cmds.stopCommand();

    cuReqs.startForeignReceive(epId, actId);
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
    else
        schedNocRequest(pkt, clockEdge(delay));
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
            delete state;
        }

        DPRINTF(TcuLLCMemAcc,
            "Finished %s request of LLC for %u bytes @ %#x\n",
            pkt->isRead() ? "read" : "write",
            pkt->getSize(), pkt->getAddr());

        if(pkt->isRead())
            printPacket(pkt);

        schedLLCResponse(pkt, true);
    }
    else if (senderState->packetType != NocPacketType::CACHE_MEM_REQ_FUNC)
    {
        cmds.setRemoteCommand(false);
        // if the current command should be aborted, just ignore the packet
        // and finish the command
        if (cmds.isCommandAborting())
        {
            freeRequest(pkt);
            scheduleCmdFinish(Cycles(1), TcuError::ABORT);
        }
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
Tcu::sendNocResponse(PacketPtr pkt, TcuError result)
{
    auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);
    senderState->result = result;

    pkt->makeResponse();

    Cycles delay = ticksToCycles(
        pkt->headerDelay + pkt->payloadDelay);

    pkt->headerDelay = 0;
    pkt->payloadDelay = 0;

    schedNocRequestFinished(clockEdge(Cycles(1)));
    schedNocResponse(pkt, clockEdge(delay));
}

void
Tcu::sendMemRequest(PacketPtr pkt,
                    Addr data,
                    Cycles delay,
                    bool coverage)
{
    auto senderState = new MemSenderState();
    senderState->data = data;
    senderState->mid = pkt->req->requestorId();
    senderState->coverage = coverage;

    pkt->pushSenderState(senderState);

    schedMemRequest(pkt, clockEdge(delay));
}

void
Tcu::completeMemRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isResponse());

    auto senderState = dynamic_cast<MemSenderState*>(pkt->popSenderState());

    // set the old requestor id again
    pkt->req->setRequestorId(senderState->mid);

    if (senderState->coverage)
    {
        auto ev = reinterpret_cast<WriteCoverageEvent*>(senderState->data);
        ev->completed(pkt);
    }
    else
        xferUnit->recvMemResponse(senderState->data, pkt);

    delete senderState;
    freeRequest(pkt);
}

bool
Tcu::handleCUMemRequest(PacketPtr pkt,
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

    Tick when = clockEdge(transportDelay + mmioLatency);

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

        if (result & RegFile::WROTE_CU_REQ)
            schedule(completeCUReqEvent, when);
        if (result & RegFile::WROTE_PRINT)
        {
            PrintReg pr = regs().get(UnprivReg::PRINT);
            if (pr.cov_addr == 0)
                printLine(pr.size);
            else
                writeCoverage(pr);
        }
        if (result & RegFile::WROTE_CLEAR_IRQ)
        {
            auto irq = (BaseConnector::IRQ)regs().get(PrivReg::CLEAR_IRQ);
            connector.clearIrq(irq);
        }
    }

    cmds.startCommand(result, pkt, when);
}

NocAddr
Tcu::translatePhysToNoC(Addr phys, bool write)
{
    Addr physAddr = phys - tileMemOffset;
    Addr physOff = physAddr & 0x3FFFFFFF;
    epid_t epid = physAddr >> 30;

    if (epid >= numEndpoints)
    {
        DPRINTFS(Tcu, this, "PMP-EP%u: invalid PMP EP (phys=%#x)\n", epid, phys);
        cuReqs.startPMPFailure(phys, write, TcuError::NO_PMP_EP);
        return NocAddr();
    }

    if (regs().getEp(epid).type() != EpType::MEMORY)
    {
        DPRINTFS(Tcu, this, "PMP-EP%u: no memory EP (phys=%#x)\n", epid, phys);
        cuReqs.startPMPFailure(phys, write, TcuError::NO_MEP);
        return NocAddr();
    }

    const MemEp mep = regs().getEp(epid).mem;

    if (physOff >= mep.r2.remoteSize)
    {
        DPRINTFS(Tcu, this,
                 "PMP-EP%u: out of bounds (%#x vs. %#x)\n",
                 epid, physOff, mep.r2.remoteSize);
        cuReqs.startPMPFailure(phys, write, TcuError::OUT_OF_BOUNDS);
        return NocAddr();
    }
    if ((!write && !(mep.r0.flags & MemoryFlags::READ)) ||
        (write && !(mep.r0.flags & MemoryFlags::WRITE)))
    {
        DPRINTFS(Tcu, this,
                 "PMP-EP%u: permission denied (flags=%#x, write=%d)\n",
                 epid, mep.r0.flags, write);
        cuReqs.startPMPFailure(phys, write, TcuError::NO_PERM);
        return NocAddr();
    }

    // translate to NoC address
    return NocAddr(TileId::from_raw(mep.r0.targetTile),
                   mep.r1.remoteAddr + physOff);
}

bool
Tcu::handleLLCRequest(PacketPtr pkt, bool functional)
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

    DPRINTF(TcuLLCMemAcc, "Sending LLC request for %#x to %s:%#x\n",
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
Tcu::printPacket(PacketPtr pkt) const
{
    DPRINTF(TcuPackets, "Dumping packet %s @ %p with %lu bytes\n",
        pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    DDUMP(TcuPackets, pkt->getPtr<uint8_t>(), pkt->getSize());
}

}
}
