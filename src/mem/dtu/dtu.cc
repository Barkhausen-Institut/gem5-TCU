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

#include <iomanip>
#include <sstream>

#include "arch/x86/m3/system.hh"
#include "debug/Dtu.hh"
#include "debug/DtuBuf.hh"
#include "debug/DtuCmd.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuSysCalls.hh"
#include "debug/DtuMem.hh"
#include "debug/DtuTlb.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/msg_unit.hh"
#include "mem/dtu/mem_unit.hh"
#include "mem/dtu/xfer_unit.hh"
#include "mem/dtu/pt_unit.hh"
#include "mem/cache/cache.hh"
#include "sim/system.hh"

static const char *cmdNames[] =
{
    "IDLE",
    "SEND",
    "REPLY",
    "READ",
    "WRITE",
    "INC_READ_PTR",
    "DEBUG_MSG",
};

static const char *extCmdNames[] =
{
    "WAKEUP_CORE",
    "INV_PAGE",
    "INV_TLB",
    "INV_CACHE",
    "INJECT_IRQ",
};

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    masterId(p->system->getMasterId(name())),
    system(p->system),
    regFile(name() + ".regFile", p->num_endpoints),
    connector(p->connector),
    tlBuf(p->tlb_entries > 0 ? new DtuTlb(*this, p->tlb_entries) : NULL),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, p->block_size, p->buf_count, p->buf_size)),
    ptUnit(p->tlb_entries > 0 ? new PtUnit(*this) : NULL),
    executeCommandEvent(*this),
    cmdInProgress(false),
    memPe(),
    memOffset(),
    atomicMode(p->system->isAtomicMode()),
    numEndpoints(p->num_endpoints),
    maxNocPacketSize(p->max_noc_packet_size),
    numCmdArgBits(32),
    numCmdFlagsBits(1),
    blockSize(p->block_size),
    bufCount(p->buf_count),
    bufSize(p->buf_size),
    cacheBlocksPerCycle(p->cache_blocks_per_cycle),
    registerAccessLatency(p->register_access_latency),
    commandToNocRequestLatency(p->command_to_noc_request_latency),
    startMsgTransferDelay(p->start_msg_transfer_delay),
    transferToMemRequestLatency(p->transfer_to_mem_request_latency),
    transferToNocLatency(p->transfer_to_noc_latency),
    nocToTransferLatency(p->noc_to_transfer_latency)
{
    assert(p->buf_size >= maxNocPacketSize);

    M3X86System *sys = dynamic_cast<M3X86System*>(system);
    if (sys)
    {
        memPe = sys->memPe;
        memOffset = sys->memOffset;
        DPRINTF(Dtu, "Using memory range %p .. %p\n",
            memOffset, memOffset + sys->memSize);

        regs().set(DtuReg::ROOT_PT, sys->getRootPt().getAddr());
    }

    regs().set(DtuReg::RW_BARRIER, -1);
    regs().set(DtuReg::VPE_ID, INVALID_VPE_ID);
}

Dtu::~Dtu()
{
    delete ptUnit;
    delete xferUnit;
    delete memUnit;
    delete msgUnit;
    delete tlBuf;
}

void
Dtu::regStats()
{
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

    commands
        .init(sizeof(cmdNames) / sizeof(cmdNames[0]))
        .name(name() + ".commands")
        .desc("The executed commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(cmdNames) / sizeof(cmdNames[0]); ++i)
        commands.subname(i, cmdNames[i]);

    extCommands
        .init(sizeof(extCmdNames) / sizeof(extCmdNames[0]))
        .name(name() + ".extCommands")
        .desc("The executed external commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(extCmdNames) / sizeof(extCmdNames[0]); ++i)
        extCommands.subname(i, extCmdNames[i]);

    if (tlb())
        tlb()->regStats();
    if (ptUnit)
        ptUnit->regStats();
    xferUnit->regStats();
    memUnit->regStats();
    msgUnit->regStats();
}

PacketPtr
Dtu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
{
    Request::Flags flags;

    auto req = new Request(paddr, size, flags, masterId);

    auto pkt = new Packet(req, cmd);
    auto pktData = new uint8_t[size];
    pkt->dataDynamic(pktData);

    return pkt;
}

void
Dtu::freeRequest(PacketPtr pkt)
{
    delete pkt->req;
    delete pkt;
}

Dtu::Command
Dtu::getCommand()
{
    assert(numCmdArgBits + numCmdFlagsBits + numCmdOpcodeBits <=
        sizeof(RegFile::reg_t) * 8);

    using reg_t = RegFile::reg_t;

    /*
     *                                          0
     * |----------------------------------------|
     * |  error  |   flags  |   arg  |  opcode  |
     * |----------------------------------------|
     */
    reg_t opcodeMask = ((reg_t)1 << numCmdOpcodeBits) - 1;
    reg_t argMask = ((reg_t)1 << numCmdArgBits) - 1;
    reg_t flagMask = ((reg_t)1 << numCmdFlagsBits) - 1;

    auto reg = regFile.get(CmdReg::COMMAND);

    Command cmd;

    unsigned bits = numCmdOpcodeBits + numCmdArgBits + numCmdFlagsBits;
    cmd.error  = static_cast<Error>(reg >> bits);

    cmd.opcode = static_cast<Command::Opcode>(reg & opcodeMask);

    cmd.arg    = (reg >> numCmdOpcodeBits) & argMask;

    cmd.flags  = (reg >> (numCmdArgBits + numCmdOpcodeBits)) & flagMask;

    return cmd;
}

void
Dtu::executeCommand()
{
    Command cmd = getCommand();
    if (cmd.opcode == Command::IDLE)
        return;

    assert(!cmdInProgress);

    cmdInProgress = true;
    commands[static_cast<size_t>(cmd.opcode)]++;

    if(cmd.opcode != Command::DEBUG_MSG)
    {
        assert(cmd.arg < numEndpoints);
        DPRINTF(DtuCmd, "Starting command %s with EP=%u, flags=%#x\n",
                cmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg, cmd.flags);
    }

    switch (cmd.opcode)
    {
    case Command::SEND:
    case Command::REPLY:
        msgUnit->startTransmission(cmd);
        break;
    case Command::READ:
        memUnit->startRead(cmd);
        break;
    case Command::WRITE:
        memUnit->startWrite(cmd);
        break;
    case Command::INC_READ_PTR:
        msgUnit->incrementReadPtr(cmd.arg);
        finishCommand(Error::NONE);
        break;
    case Command::DEBUG_MSG:
        DPRINTF(Dtu, "DEBUG %#x\n", cmd.arg);
        finishCommand(Error::NONE);
        break;
    default:
        // TODO error handling
        panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }
}

void
Dtu::finishCommand(Error error)
{
    Command cmd = getCommand();

    assert(cmdInProgress);

    DPRINTF(DtuCmd, "Finished command %s with EP=%u, flags=%#x -> %u\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg, cmd.flags,
            static_cast<uint>(error));

    // let the SW know that the command is finished
    unsigned bits = numCmdOpcodeBits + numCmdFlagsBits + numCmdArgBits;
    regFile.set(CmdReg::COMMAND, static_cast<uint>(error) << bits);

    cmdInProgress = false;
}

Dtu::ExternCommand
Dtu::getExternCommand()
{
    auto reg = regFile.get(DtuReg::EXT_CMD);

    ExternCommand cmd;
    cmd.opcode = static_cast<ExternCommand::Opcode>(reg & 0x7);
    cmd.arg = reg >> 3;
    return cmd;
}

void
Dtu::executeExternCommand(PacketPtr pkt)
{
    ExternCommand cmd = getExternCommand();

    DPRINTF(DtuCmd, "Executing extern command %s with arg=%p\n",
            extCmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg);

    extCommands[static_cast<size_t>(cmd.opcode)]++;

    Cycles delay(1);

    switch (cmd.opcode)
    {
    case ExternCommand::WAKEUP_CORE:
        wakeupCore();
        break;
    case ExternCommand::INV_PAGE:
        if (tlb())
            tlb()->remove(cmd.arg);
        break;
    case ExternCommand::INV_TLB:
        if (tlb())
            tlb()->clear();
        break;
    case ExternCommand::INV_CACHE:
        delay = Cycles(0);
        if(l1Cache)
        {
            l1Cache->memInvalidate();
            delay += Cycles(l1Cache->getBlockCount() / cacheBlocksPerCycle);
        }
        if(l2Cache)
        {
            l2Cache->memInvalidate();
            delay += Cycles(l2Cache->getBlockCount() / cacheBlocksPerCycle);
        }
        break;
    case ExternCommand::INJECT_IRQ:
        injectIRQ(cmd.arg);
        break;
    default:
        // TODO error handling
        panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (pkt)
        schedNocResponse(pkt, clockEdge(delay));
}

void
Dtu::wakeupCore()
{
    connector->wakeup();
}

void
Dtu::updateSuspendablePin()
{
    connector->suspend(regFile.get(DtuReg::MSG_CNT));
}

void
Dtu::injectIRQ(int vector)
{
    connector->injectIrq(vector);

    irqInjects++;
}

void
Dtu::sendMemRequest(PacketPtr pkt,
                    Addr virt,
                    Addr data,
                    MemReqType type,
                    Cycles delay)
{
    auto senderState = new MemSenderState();
    senderState->data = data;
    senderState->mid = pkt->req->masterId();
    senderState->type = type;

    regWatchRange(pkt, virt);

    // ensure that this packet has our master id (not the id of a master in
    // a different PE)
    pkt->req->setMasterId(masterId);

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
Dtu::sendNocRequest(NocPacketType type,
                    PacketPtr pkt,
                    uint vpeId,
                    uint flags,
                    Cycles delay,
                    bool functional)
{
    auto senderState = new NocSenderState();
    senderState->packetType = type;
    senderState->result = Error::NONE;
    senderState->vpeId = vpeId;
    senderState->flags = flags;

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
Dtu::sendNocResponse(PacketPtr pkt)
{
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
Dtu::startTransfer(TransferType type,
                   NocAddr targetAddr,
                   Addr sourceAddr,
                   Addr size,
                   PacketPtr pkt,
                   uint vpeId,
                   MessageHeader* header,
                   Cycles delay,
                   uint flags)
{
    xferUnit->startTransfer(type,
                            targetAddr,
                            sourceAddr,
                            size,
                            pkt,
                            vpeId,
                            header,
                            delay,
                            flags);
}

void
Dtu::finishMsgReceive(unsigned epId)
{
    msgUnit->incrementMsgCnt(epId);
}

void
Dtu::startTranslate(Addr virt,
                    uint access,
                    PtUnit::Translation *trans)
{
    ptUnit->startTranslate(virt, access, trans);
}

void
Dtu::handlePFResp(PacketPtr pkt)
{
    ptUnit->finishPagefault(pkt);
}

void
Dtu::completeNocRequest(PacketPtr pkt)
{
    auto senderState = dynamic_cast<NocSenderState*>(pkt->popSenderState());

    if (senderState->packetType == NocPacketType::CACHE_MEM_REQ)
    {
        NocAddr phys(pkt->getAddr());
        DPRINTF(DtuMem,
            "Finished %s request of LLC for %u bytes @ %d:%#x -> %u\n",
            pkt->isRead() ? "read" : "write",
            pkt->getSize(), phys.coreId, phys.offset,
            static_cast<uint>(senderState->result));

        if (dynamic_cast<InitSenderState*>(pkt->senderState))
        {
            // undo the change from handleCacheMemRequest
            pkt->setAddr(phys.offset - memOffset);
            pkt->req->setPaddr(phys.offset - memOffset);
            pkt->popSenderState();
        }

        if (senderState->result != Error::NONE)
        {
            uint access = DtuTlb::INTERN | DtuTlb::GONE;
            VPEGoneTranslation *trans = new VPEGoneTranslation(*this, pkt);
            ptUnit->startTranslate(pkt->getAddr(), access, trans);
        }
        else
            sendCacheMemResponse(pkt, true);
    }
    else if (senderState->packetType == NocPacketType::PAGEFAULT)
    {
        if (senderState->result != Error::NONE)
            ptUnit->sendingPfFailed(pkt, static_cast<int>(senderState->result));
    }
    else if (senderState->packetType != NocPacketType::CACHE_MEM_REQ_FUNC)
    {
        if (pkt->isWrite())
            memUnit->writeComplete(pkt, senderState->result);
        else if (pkt->isRead())
            memUnit->readComplete(pkt, senderState->result);
        else
            panic("unexpected packet type\n");
    }

    delete senderState;
}

void
Dtu::completeMemRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isResponse());

    auto senderState = dynamic_cast<MemSenderState*>(pkt->popSenderState());

    // set the old master id again
    pkt->req->setMasterId(senderState->mid);

    switch(senderState->type)
    {
    case MemReqType::TRANSFER:
        xferUnit->recvMemResponse(senderState->data,
                                  pkt->getConstPtr<uint8_t>(),
                                  pkt->getSize(),
                                  pkt->headerDelay,
                                  pkt->payloadDelay);
        break;

    case MemReqType::HEADER:
        msgUnit->recvFromMem(getCommand(), pkt);
        break;

    case MemReqType::TRANSLATION:
        ptUnit->recvFromMem(senderState->data, pkt);
        break;
    }

    checkWatchRange(pkt);

    delete senderState;
    freeRequest(pkt);
}

void
Dtu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());

    auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);

    Error res = Error::NONE;

    switch (senderState->packetType)
    {
    case NocPacketType::MESSAGE:
    case NocPacketType::PAGEFAULT:
        nocMsgRecvs++;
        res = msgUnit->recvFromNoc(pkt, senderState->vpeId);
        break;
    case NocPacketType::READ_REQ:
    case NocPacketType::WRITE_REQ:
    case NocPacketType::CACHE_MEM_REQ:
        if (senderState->packetType == NocPacketType::READ_REQ)
            nocReadRecvs++;
        else if (senderState->packetType == NocPacketType::WRITE_REQ)
            nocWriteRecvs++;
        res = memUnit->recvFromNoc(pkt, senderState->vpeId, senderState->flags);
        break;
    case NocPacketType::CACHE_MEM_REQ_FUNC:
        memUnit->recvFunctionalFromNoc(pkt);
        break;
    default:
        panic("Unexpected NocPacketType\n");
    }

    senderState->result = res;
}

bool
Dtu::handleCpuRequest(PacketPtr pkt,
                      DtuSlavePort &sport,
                      DtuMasterPort &mport,
                      bool icache,
                      bool functional)
{
    bool res = true;

    if (pkt->getAddr() >= regFileBaseAddr)
    {
        // not supported here
        assert(!functional);

        if (icache)
            res = false;
        else
            forwardRequestToRegFile(pkt, true);
    }
    else if(pkt->isWrite() &&
            pkt->getAddr() >= regFile.get(DtuReg::RW_BARRIER))
    {
        DPRINTF(Dtu, "Warning: ignoring write access above rwBarrier\n");
        res = false;
    }
    else
    {
        regWatchRange(pkt, pkt->getAddr());

        MemTranslation *trans = new MemTranslation(*this, sport, mport, pkt);
        int tres = translate(trans, pkt, icache, functional);
        if (tres == 1)
        {
            intMemReqs++;
            if (functional)
                mport.sendFunctional(pkt);
            else
                mport.schedTimingReq(pkt, curTick());
            delete trans;
        }
        else if (tres == -1)
        {
            res = false;
            delete trans;
        }
    }

    return res;
}

bool
Dtu::handleCacheMemRequest(PacketPtr pkt, bool functional)
{
    if (pkt->cmd == MemCmd::CleanEvict)
    {
        assert(!pkt->needsResponse());
        DPRINTF(DtuPackets, "Dropping CleanEvict packet\n");
        return true;
    }

    // we don't have cache coherence. so we don't care about invalidate req.
    if (pkt->cmd == MemCmd::InvalidateReq)
        return false;
    if (pkt->cmd == MemCmd::BadAddressError)
        return false;

    Addr old = pkt->getAddr();
    NocAddr phys(pkt->getAddr());
    // special case: we check whether this is actually a NocAddr. this does
    // only happen when loading a program at startup, TLB misses in the core
    // and pseudoInst
    if (!phys.valid)
    {
        phys = NocAddr(memPe, memOffset + phys.offset);
        pkt->setAddr(phys.getAddr());
        if (!functional)
        {
            // remember that we did this change
            pkt->pushSenderState(new InitSenderState);
        }
    }

    DPRINTF(DtuMem, "Handling %s request of LLC for %u bytes @ %d:%#x\n",
                    pkt->isRead() ? "read" : "write",
                    pkt->getSize(), phys.coreId, phys.offset);

    auto type = functional ? Dtu::NocPacketType::CACHE_MEM_REQ_FUNC
                           : Dtu::NocPacketType::CACHE_MEM_REQ;
    // this does always target a memory PE, so vpeId is invalid
    sendNocRequest(type,
                   pkt,
                   INVALID_VPE_ID,
                   Command::NONE,
                   Cycles(1),
                   functional);

    extMemReqs++;

    if (functional)
        pkt->setAddr(old);

    return true;
}

int
Dtu::translate(PtUnit::Translation *trans,
               PacketPtr pkt,
               bool icache,
               bool functional)
{
    if (!tlb())
        return 1;

    uint access = DtuTlb::INTERN;
    if (icache)
    {
        assert(pkt->isRead());
        access |= DtuTlb::EXEC;
    }
    else if (pkt->isRead())
        access |= DtuTlb::READ;
    else
        access |= DtuTlb::WRITE;

    NocAddr phys;
    DtuTlb::Result res = tlb()->lookup(pkt->getAddr(), access, &phys);
    switch(res)
    {
        case DtuTlb::HIT:
            DPRINTF(DtuTlb, "Translated %s access for %p -> %p\n",
                    icache ? "exec" : (pkt->isRead() ? "read" : "write"),
                    pkt->getAddr(), phys.getAddr());

            pkt->setAddr(phys.getAddr());
            pkt->req->setPaddr(phys.getAddr());
            break;

        case DtuTlb::NOMAP:
        case DtuTlb::MISS:
        case DtuTlb::PAGEFAULT:
        {
            bool pf = res != DtuTlb::MISS;
            DPRINTF(DtuTlb, "%s%s for %s access to %p\n",
                    pf ? "Pagefault" : "TLB-miss",
                    res == DtuTlb::NOMAP ? " (ignored)" : "",
                    icache ? "exec" : (pkt->isRead() ? "read" : "write"),
                    pkt->getAddr());

            // don't cause a pagefault again in this case
            if (res == DtuTlb::NOMAP)
                return -1;

            if (functional)
            {
                int res = 0;
                if (!pf)
                {
                    NocAddr phys;
                    res = ptUnit->translateFunctional(pkt->getAddr(),
                                                      access,
                                                      &phys);
                    if (res)
                    {
                        pkt->setAddr(phys.getAddr());
                        pkt->req->setPaddr(phys.getAddr());
                    }
                }

                return !res ? -1 : 1;
            }

            ptUnit->startTranslate(pkt->getAddr(), access, trans);
        }
        return 0;
    }

    return 1;
}

void
Dtu::forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest)
{
    Addr oldAddr = pkt->getAddr();

    // Strip the base address to handle requests based on the reg. addr. only.
    pkt->setAddr(oldAddr - regFileBaseAddr);

    RegFile::Result result = regFile.handleRequest(pkt, isCpuRequest);

    regFileReqs++;

    // restore old address
    pkt->setAddr(oldAddr);

    updateSuspendablePin();

    if (!atomicMode)
    {
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
            pkt->headerDelay = 0;
            pkt->payloadDelay = 0;

            if (isCpuRequest)
                schedCpuResponse(pkt, when);
            else
                schedNocResponse(pkt, when);

            if (result & RegFile::WROTE_CMD)
                schedule(executeCommandEvent, when);
        }
        else
            schedule(new ExecExternCmdEvent(*this, pkt), when);
    }
    else
    {
        if (result & RegFile::WROTE_CMD)
            executeCommand();
        if (result & RegFile::WROTE_EXT_CMD)
            executeExternCommand(NULL);
    }
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}

void
Dtu::printPacket(PacketPtr pkt) const
{
    DDUMP(DtuPackets, pkt->getPtr<uint8_t>(), pkt->getSize());
}
