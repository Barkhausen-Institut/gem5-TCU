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

#include "debug/Dtu.hh"
#include "debug/DtuBuf.hh"
#include "debug/DtuCmd.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuMem.hh"
#include "debug/DtuCoreMemAcc.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/msg_unit.hh"
#include "mem/dtu/mem_unit.hh"
#include "mem/dtu/xfer_unit.hh"
#include "mem/cache/cache.hh"
#include "sim/pe_memory.hh"
#include "sim/system.hh"

static const char *cmdNames[] =
{
    "IDLE",
    "SEND",
    "REPLY",
    "READ",
    "WRITE",
    "FETCH_MSG",
    "FETCH_EVENTS",
    "SET_EVENT",
    "ACK_MSG",
    "SLEEP",
    "PRINT",
};

static const char *privCmdNames[] =
{
    "IDLE",
    "INV_EP",
    "INV_PAGE",
    "INV_TLB",
    "INV_REPLY",
    "RESET",
    "FLUSH_CACHE",
    "XCHG_VPE",
};

// cmdId = 0 is reserved for "no command"
// cmdId = 1 is reserved for a dummy command (waiting for remote xfers)
uint64_t Dtu::nextCmdId = 2;

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    masterId(p->system->getMasterId(this, name())),
    system(p->system),
    regFile(*this, name() + ".regFile", p->num_endpoints),
    connector(p->connector),
    tlBuf(p->tlb_entries > 0 ? new DtuTlb(*this, p->tlb_entries) : NULL),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, p->block_size, p->buf_count, p->buf_size)),
    coreReqs(*this, p->buf_count),
    abortCommandEvent(*this),
    completeCoreReqEvent(coreReqs),
    cmdPkt(),
    cmdFinish(),
    cmdId(0),
    abortCmd(0),
    cmdXferBuf(0),
    cmdSent(),
    wakeupEp(0xFFFF),
    memPe(),
    memOffset(),
    atomicMode(p->system->isAtomicMode()),
    numEndpoints(p->num_endpoints),
    maxNocPacketSize(p->max_noc_packet_size),
    blockSize(p->block_size),
    bufCount(p->buf_count),
    bufSize(p->buf_size),
    reqCount(p->req_count),
    cacheBlocksPerCycle(p->cache_blocks_per_cycle),
    registerAccessLatency(p->register_access_latency),
    cpuToCacheLatency(p->cpu_to_cache_latency),
    commandToNocRequestLatency(p->command_to_noc_request_latency),
    startMsgTransferDelay(p->start_msg_transfer_delay),
    transferToMemRequestLatency(p->transfer_to_mem_request_latency),
    transferToNocLatency(p->transfer_to_noc_latency),
    nocToTransferLatency(p->noc_to_transfer_latency)
{
    assert(p->buf_size >= maxNocPacketSize);

    PEMemory *sys = dynamic_cast<PEMemory*>(system);
    if (sys)
    {
        NocAddr phys = sys->getPhys(0);
        memPe = phys.coreId;
        memOffset = phys.offset;
        memSize = sys->memSize;
        DPRINTF(Dtu, "Using memory range %p .. %p\n",
            memOffset, memOffset + memSize);
    }
    // memory PEs don't participate in cache coherence
    else
        coherent = false;
}

Dtu::~Dtu()
{
    delete xferUnit;
    delete memUnit;
    delete msgUnit;
    delete tlBuf;
}

void
Dtu::regStats()
{
    BaseDtu::regStats();

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

    commands
        .init(sizeof(cmdNames) / sizeof(cmdNames[0]))
        .name(name() + ".commands")
        .desc("The executed commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(cmdNames) / sizeof(cmdNames[0]); ++i)
        commands.subname(i, cmdNames[i]);

    privCommands
        .init(sizeof(privCmdNames) / sizeof(privCmdNames[0]))
        .name(name() + ".privCommands")
        .desc("The executed privileged commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(privCmdNames) / sizeof(privCmdNames[0]); ++i)
        privCommands.subname(i, privCmdNames[i]);

    if (tlb())
        tlb()->regStats();
    coreReqs.regStats();
    xferUnit->regStats();
    memUnit->regStats();
    msgUnit->regStats();
}

bool
Dtu::isMemPE(unsigned pe) const
{
    PEMemory *sys = dynamic_cast<PEMemory*>(system);
    return !sys || sys->hasMem(pe);
}

PacketPtr
Dtu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
{
    Request::Flags flags;

    auto req = std::make_shared<Request>(paddr, size, flags, masterId);

    auto pkt = new Packet(req, cmd);

    if (size)
    {
        auto pktData = new uint8_t[size];
        pkt->dataDynamic(pktData);
    }

    return pkt;
}

void
Dtu::freeRequest(PacketPtr pkt)
{
    delete pkt;
}

void
Dtu::executeCommand(PacketPtr pkt)
{
    Command::Bits cmd = getCommand();
    if (cmd.opcode == Command::IDLE)
    {
        if (pkt)
            schedCpuResponse(pkt, clockEdge(Cycles(1)));
        return;
    }

    assert(cmdId == 0);

    cmdPkt = pkt;
    cmdId = nextCmdId++;
    cmdSent = false;
    commands[static_cast<size_t>(cmd.opcode)]++;

    assert(cmd.epid < numEndpoints);
    DPRINTF(DtuCmd, "Starting command %s with EP=%u, flags=%#x arg=%#lx (id=%llu)\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid,
            cmd.flags, cmd.arg, cmdId);

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
        case Command::FETCH_MSG:
            regs().set(CmdReg::ARG1, msgUnit->fetchMessage(cmd.epid));
            finishCommand(DtuError::NONE);
            break;
        case Command::FETCH_EVENTS:
            regs().set(CmdReg::ARG1, regs().fetchEvents());
            finishCommand(DtuError::NONE);
            break;
        case Command::SET_EVENT:
            regs().setEvent(EventType::USER);
            finishCommand(DtuError::NONE);
            break;
        case Command::ACK_MSG:
            finishCommand(msgUnit->ackMessage(cmd.epid, cmd.arg));
            break;
        case Command::SLEEP:
        {
            RegFile::reg_t arg = regs().get(CmdReg::ARG1);
            Cycles sleepCycles = static_cast<Cycles>(arg & 0xFFFFFFFFFFF);
            int ep = (arg >> 48) & 0xFFFF;
            bool ackEvents = cmd.arg != 0;
            if (!startSleep(sleepCycles, ep, ackEvents))
                finishCommand(DtuError::NONE);
        }
        break;
        case Command::PRINT:
        {
            printLine(cmd.arg);
            finishCommand(DtuError::NONE);
        }
        break;
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if(cmd.opcode == Command::SEND || cmd.opcode == Command::REPLY ||
       cmd.opcode == Command::READ || cmd.opcode == Command::WRITE)
    {
        schedCpuResponse(cmdPkt, clockEdge(Cycles(1)));
        cmdPkt = nullptr;
    }
}

void
Dtu::abortCommand()
{
    Command::Bits cmd = getCommand();
    abortCmd = regs().get(CmdReg::ABORT);

    if (cmd.opcode != Command::IDLE)
    {
        DPRINTF(DtuCmd, "Aborting command %s with EP=%u, flags=%#x (id=%llu)\n",
                cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid,
                cmd.flags, cmdId);
    }

    cmdXferBuf = -1;
    xferUnit->abortTransfers(XferUnit::ABORT_LOCAL);

    regs().set(CmdReg::ABORT, cmdXferBuf);

    // if no command was running, let the SW wait until the currently received
    // messages are finished
    DtuError err = static_cast<DtuError>(static_cast<uint>(cmd.error));
    if (cmd.opcode == Command::IDLE)
        abortCmd = 0;
    // reads/writes are aborted immediately
    else if(cmd.opcode == Command::READ || cmd.opcode == Command::WRITE)
    {
        cmdId = 0;
        err = DtuError::ABORT;
    }
    // message sends are aborted, if they haven't been sent yet
    else if (!cmdSent && (cmd.opcode == Command::SEND ||
                          cmd.opcode == Command::REPLY))
    {
        err = DtuError::ABORT;
    }

    if (cmd.opcode == Command::IDLE ||
        cmd.opcode == Command::READ ||
        cmd.opcode == Command::WRITE ||
        !cmdSent)
        scheduleFinishOp(Cycles(1), err);
}

void
Dtu::scheduleFinishOp(Cycles delay, DtuError error)
{
    if (getCommand().opcode != Command::IDLE)
    {
        if (cmdFinish)
        {
            deschedule(cmdFinish);
            delete cmdFinish;
        }

        cmdFinish = new FinishCommandEvent(*this, error);
        schedule(cmdFinish, clockEdge(delay));
    }
}

void
Dtu::finishCommand(DtuError error)
{
    Command::Bits cmd = getCommand();

    cmdFinish = NULL;

    if (abortCmd)
    {
        // try to abort everything
        if (!xferUnit->abortTransfers(XferUnit::ABORT_LOCAL))
        {
            // okay, retry later
            scheduleFinishOp(Cycles(1), error);
            return;
        }

        abortCmd = 0;
    }

    if (cmd.opcode == Command::SEND)
        msgUnit->finishMsgSend(error, cmd.epid);
    else if (cmd.opcode == Command::REPLY)
        msgUnit->finishMsgReply(error, cmd.epid, cmd.arg);
    else if (error == DtuError::NONE &&
             (cmd.opcode == Command::READ || cmd.opcode == Command::WRITE))
    {
        const DataReg data = regs().getDataReg();
        if (data.size > 0)
        {
            if (cmd.opcode == Command::READ)
                memUnit->startRead(cmd);
            else
                memUnit->startWrite(cmd);
            return;
        }
    }

    if (cmd.opcode == Command::SLEEP)
        stopSleep();

    DPRINTF(DtuCmd, "Finished command %s with EP=%u, flags=%#x (id=%llu) -> %u\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid, cmd.flags,
            cmdId, static_cast<uint>(error));

    // let the SW know that the command is finished
    cmd = 0;
    cmd.error = static_cast<unsigned>(error);
    cmd.opcode = Command::IDLE;
    regFile.set(CmdReg::COMMAND, cmd);

    if (cmdPkt)
        schedCpuResponse(cmdPkt, clockEdge(Cycles(1)));

    cmdPkt = NULL;
    cmdId = 0;
}

Dtu::PrivCommand
Dtu::getPrivCommand()
{
    auto reg = regFile.get(PrivReg::PRIV_CMD);

    PrivCommand cmd;
    cmd.opcode = static_cast<PrivCommand::Opcode>(reg & 0xF);
    cmd.arg = reg >> 4;
    return cmd;
}

void
Dtu::executePrivCommand(PacketPtr pkt)
{
    PrivCommand cmd = getPrivCommand();

    privCommands[static_cast<size_t>(cmd.opcode)]++;

    Cycles delay(1);

    DtuError result = DtuError::NONE;

    DPRINTF(DtuCmd, "Executing privileged command %s with arg=%p\n",
            privCmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg);

    switch (cmd.opcode)
    {
        case PrivCommand::IDLE:
            break;
        case PrivCommand::INV_EP:
        {
            unsigned epid = cmd.arg & 0xFFFF;
            bool force = !!(cmd.arg & (1 << 16));
            if (!regs().invalidate(epid, force))
                result = DtuError::MISS_CREDITS;
            else {
                regs().setEvent(EventType::EP_INVAL);
                wakeupCore(false);
            }
            break;
        }
        case PrivCommand::INV_PAGE:
            if (tlb())
            {
                uint16_t asid = cmd.arg >> 44;
                Addr virt = cmd.arg & 0xFFFFFFFFFFF;
                tlb()->remove(virt, asid);
            }
            break;
        case PrivCommand::INV_TLB:
            if (tlb())
                tlb()->clear();
            break;
        case PrivCommand::INV_REPLY:
        {
            unsigned repid = cmd.arg & 0xFFFF;
            unsigned peid = (cmd.arg >> 16) & 0xFF;
            unsigned sepid = (cmd.arg >> 24) & 0xFFFF;
            result = msgUnit->invalidateReply(repid, peid, sepid);
            break;
        }
        case PrivCommand::RESET:
            delay += reset(!!(cmd.arg >> 59));
            break;
        case PrivCommand::FLUSH_CACHE:
            delay += flushInvalCaches(false);
            break;
        case PrivCommand::XCHG_VPE:
        {
            RegFile::reg_t old = regs().get(PrivReg::CUR_VPE);
            regs().set(PrivReg::OLD_VPE, old);
            regs().set(PrivReg::CUR_VPE, cmd.arg & 0x7FFFFFFFF);
            break;
        }
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (pkt)
    {
        auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);
        if (senderState)
        {
            senderState->result = result;
            schedNocResponse(pkt, clockEdge(delay));
        }
        else
        {
            schedCpuResponse(pkt, clockEdge(Cycles(1)));
        }
    }

    if (result != DtuError::NONE)
    {
        DPRINTF(DtuCmd, "Privileged command %s failed (%u)\n",
                privCmdNames[static_cast<size_t>(cmd.opcode)],
                (unsigned)result);
    }

    // set privileged command back to IDLE
    regFile.set(PrivReg::PRIV_CMD,
        static_cast<RegFile::reg_t>(PrivCommand::IDLE));
}

bool
Dtu::has_message(int ep)
{
    return (ep == 0xFFFF && regFile.messages() > 0) ||
           (ep != 0xFFFF && regFile.getRecvEp(ep).unread != 0);
}

bool
Dtu::startSleep(uint64_t cycles, int ep, bool ack)
{
    // just for accelerators and simplicity: ack all events
    if (ack)
        regFile.fetchEvents();

    if (regFile.hasEvents() || has_message(ep))
        return false;
    if (regFile.get(PrivReg::CORE_REQ) != 0)
        return false;

    wakeupEp = ep;
    DPRINTF(Dtu, "Suspending CU (waiting for EP %d)\n", wakeupEp);
    connector->suspend();

    if (cycles)
        scheduleFinishOp(Cycles(cycles));
    return true;
}

void
Dtu::stopSleep()
{
    connector->wakeup();
}

void
Dtu::wakeupCore(bool force)
{
    if (force || regFile.hasEvents() || has_message(wakeupEp))
    {
        if (getCommand().opcode == Command::SLEEP)
            scheduleFinishOp(Cycles(1));
        else
            connector->wakeup();
    }
}

Cycles
Dtu::reset(bool flushInval)
{
    Cycles delay = flushInval ? flushInvalCaches(true) : Cycles(0);

    // hard-abort everything
    xferUnit->abortTransfers(XferUnit::ABORT_LOCAL | XferUnit::ABORT_MSGS);

    if (tlb())
        tlb()->clear();

    connector->reset();

    resets++;
    return delay;
}

Cycles
Dtu::flushInvalCaches(bool invalidate)
{
    Cycles delay(0);
    if(!coherent)
    {
        for (auto &c : caches)
        {
            c->memWriteback();
            if (invalidate)
                c->memInvalidate();
            delay += Cycles(c->getBlockCount() / cacheBlocksPerCycle);
        }
    }
    return delay;
}

void
Dtu::setIrq()
{
    wakeupCore(true);

    connector->setIrq();

    irqInjects++;
}

void
Dtu::clearIrq()
{
    connector->clearIrq();
}

void
Dtu::printLine(Addr len)
{
    const char *buffer = regs().getBuffer(len);
    DPRINTF(Dtu, "PRINT: %s\n", buffer);
}

void
Dtu::sendMemRequest(PacketPtr pkt,
                    Addr virt,
                    Addr data,
                    Cycles delay)
{
    auto senderState = new MemSenderState();
    senderState->data = data;
    senderState->mid = pkt->req->masterId();

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
                    uint flags,
                    Cycles delay,
                    bool functional)
{
    auto senderState = new NocSenderState();
    senderState->packetType = type;
    senderState->result = DtuError::NONE;
    senderState->flags = flags;
    if (regFile.hasFeature(Features::PRIV))
        senderState->flags |= NocFlags::PRIV;
    senderState->cmdId = cmdId;

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

Addr
Dtu::physToNoc(Addr phys)
{
#if THE_ISA == X86_ISA
    return (phys & ~0x0000FF0000000000ULL) |
          ((phys & 0x0000FF0000000000ULL) << 16);
#elif THE_ISA == ARM_ISA
    return (phys & ~0x000000FF00000000ULL) |
          ((phys & 0x000000FF00000000ULL) << 24);
#elif THE_ISA == RISCV_ISA
    return (phys & ~0x00FF000000000000ULL) |
          ((phys & 0x00FF000000000000ULL) << 8);
#else
#   error "Unsupported ISA"
#endif
}

Addr
Dtu::nocToPhys(Addr noc)
{
#if THE_ISA == X86_ISA
    return (noc & ~0xFF00000000000000ULL) |
          ((noc & 0xFF00000000000000ULL) >> 16);
#elif THE_ISA == ARM_ISA
    return (noc & ~0xFF00000000000000ULL) |
          ((noc & 0xFF00000000000000ULL) >> 24);
#elif THE_ISA == RISCV_ISA
    return (noc & ~0xFF00000000000000ULL) |
          ((noc & 0xFF00000000000000ULL) >> 8);
#else
#   error "Unsupported ISA"
#endif
}

void
Dtu::startTransfer(void *event, Cycles delay)
{
    auto ev = reinterpret_cast<XferUnit::TransferEvent*>(event);
    xferUnit->startTransfer(ev, delay);
}

void
Dtu::startTranslate(size_t id,
                    unsigned vpeId,
                    Addr virt,
                    uint access,
                    XferUnit::Translation *trans)
{
    coreReqs.startTranslate(id, vpeId, virt, access, trans);
}

void
Dtu::startForeignReceive(size_t id,
                         unsigned epId,
                         unsigned vpeId,
                         XferUnit::TransferEvent *event)
{
    coreReqs.startForeignReceive(id, epId, vpeId, event);
}

void
Dtu::abortTranslate(size_t id)
{
    coreReqs.abortReq(id);
    cmdXferBuf = id;
}

void
Dtu::completeNocRequest(PacketPtr pkt)
{
    auto senderState = dynamic_cast<NocSenderState*>(pkt->popSenderState());

    if (senderState->packetType == NocPacketType::CACHE_MEM_REQ)
    {
        // as these target memory PEs, there can't be any error
        assert(senderState->result == DtuError::NONE);

        NocAddr noc(pkt->getAddr());
        DPRINTF(DtuMem,
            "Finished %s request of LLC for %u bytes @ %d:%#x\n",
            pkt->isRead() ? "read" : "write",
            pkt->getSize(), noc.coreId, noc.offset);

        if(pkt->isRead())
            printPacket(pkt);

        if (dynamic_cast<InitSenderState*>(pkt->senderState))
        {
            // undo the change from handleCacheMemRequest
            pkt->setAddr(noc.offset - memOffset);
            pkt->req->setPaddr(noc.offset - memOffset);
            pkt->popSenderState();
        }

        // translate NoC address to physical address
        Addr phys = nocToPhys(pkt->getAddr());
        pkt->setAddr(phys);
        pkt->req->setPaddr(phys);

        sendCacheMemResponse(pkt, true);
    }
    else if (senderState->packetType != NocPacketType::CACHE_MEM_REQ_FUNC)
    {
        // ignore responses for aborted commands
        if (senderState->cmdId == cmdId)
        {
            if (pkt->isWrite())
                memUnit->writeComplete(getCommand(), pkt, senderState->result);
            else if (pkt->isRead())
                memUnit->readComplete(getCommand(), pkt, senderState->result);
            else
                panic("unexpected packet type\n");
        }
        else
        {
            DPRINTF(Dtu,"Ignoring response for cmdId=%llu (current=%llu)\n",
                    senderState->cmdId, cmdId);
        }
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

    xferUnit->recvMemResponse(senderState->data, pkt);

    delete senderState;
    freeRequest(pkt);
}

void
Dtu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());

    if (pkt->cacheResponding())
    {
        DPRINTF(DtuPackets, "Ignoring packet, because cache is responding\n");
        schedNocRequestFinished(clockEdge(Cycles(1)));
        return;
    }

    auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);

    DtuError res = DtuError::NONE;

    switch (senderState->packetType)
    {
        case NocPacketType::MESSAGE:
        {
            nocMsgRecvs++;
            uint flags = senderState->flags;
            res = msgUnit->recvFromNoc(pkt, flags);
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
            uint flags = senderState->flags;
            res = memUnit->recvFromNoc(pkt, flags);
            break;
        }
        case NocPacketType::CACHE_MEM_REQ_FUNC:
            memUnit->recvFunctionalFromNoc(pkt);
            break;
        default:
            panic("Unexpected NocPacketType\n");
    }

    senderState->result = res;
}

bool
Dtu::handleCoreMemRequest(PacketPtr pkt,
                          DtuSlavePort &sport,
                          DtuMasterPort &mport,
                          bool icache,
                          bool functional)
{
    bool res = true;
    bool delayed = false;
    Addr virt = pkt->getAddr();

    DPRINTF(DtuCoreMemAcc, "%s access for %#lx: start\n",
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

    if (!delayed)
    {
        DPRINTF(DtuCoreMemAcc, "%s access for %#lx: finished\n",
            pkt->cmdString(), virt);
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

    Addr physAddr = pkt->getAddr();

    // translate physical address to NoC address
    Addr nocAddr = physToNoc(physAddr);
    pkt->setAddr(nocAddr);

    NocAddr noc(nocAddr);
    // special case: we check whether this is actually a NocAddr. this does
    // only happen when loading a program at startup, TLB misses in the core
    // and pseudoInst
    if (!noc.valid)
    {
        if (noc.offset > memSize)
        {
            DPRINTF(DtuMem, "Ignoring %s request of LLC for %u bytes @ %d:%#x\n",
                pkt->cmdString(), pkt->getSize(), noc.coreId, noc.offset);
            return false;
        }

        noc = NocAddr(memPe, memOffset + noc.offset);
        pkt->setAddr(noc.getAddr());
        if (!functional)
        {
            // remember that we did this change
            pkt->pushSenderState(new InitSenderState);
        }
    }

    DPRINTF(DtuMem, "Handling %s request of LLC for %u bytes @ %d:%#x\n",
                    pkt->cmdString(),
                    pkt->getSize(), noc.coreId, noc.offset);

    if(pkt->isWrite())
        printPacket(pkt);

    auto type = functional ? Dtu::NocPacketType::CACHE_MEM_REQ_FUNC
                           : Dtu::NocPacketType::CACHE_MEM_REQ;
    // this does always target a memory PE, so vpeId is invalid
    sendNocRequest(type,
                   pkt,
                   NocFlags::NONE,
                   Cycles(1),
                   functional);

    extMemReqs++;

    if (functional)
        pkt->setAddr(physAddr);

    return true;
}

void
Dtu::forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest)
{
    Addr oldAddr = pkt->getAddr();

    // Strip the base address to handle requests based on the reg. addr. only.
    pkt->setAddr(oldAddr - mmioRegion.start());

    RegFile::Result result = regFile.handleRequest(pkt, isCpuRequest);

    regFileReqs++;

    // restore old address
    pkt->setAddr(oldAddr);

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

        if (~result & RegFile::WROTE_PRIV_CMD)
        {
            pkt->headerDelay = 0;
            pkt->payloadDelay = 0;

            if (isCpuRequest && (~result & RegFile::WROTE_CMD))
                schedCpuResponse(pkt, when);
            else if(!isCpuRequest)
                schedNocResponse(pkt, when);

            if (result & RegFile::WROTE_CMD)
                schedule(new ExecCmdEvent(*this, pkt), when);
            else if (result & RegFile::WROTE_ABORT)
                schedule(abortCommandEvent, when);
            else if (result & RegFile::WROTE_XLATE)
                schedule(completeCoreReqEvent, when);
            if (result & RegFile::WROTE_CLEAR_IRQ)
                clearIrq();
        }
        else
            schedule(new ExecPrivCmdEvent(*this, pkt), when);
    }
    else
    {
        if (result & RegFile::WROTE_CMD)
            executeCommand(NULL);
        if (result & RegFile::WROTE_PRIV_CMD)
            executePrivCommand(NULL);
        if (result & RegFile::WROTE_ABORT)
            abortCommand();
        if (result & RegFile::WROTE_XLATE)
            coreReqs.completeReqs();
        if (result & RegFile::WROTE_CLEAR_IRQ)
            clearIrq();
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
    DPRINTF(DtuPackets, "Dumping packet %s @ %p with %lu bytes\n",
        pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    DDUMP(DtuPackets, pkt->getPtr<uint8_t>(), pkt->getSize());
}
