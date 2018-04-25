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
#include "debug/DtuSysCalls.hh"
#include "debug/DtuMem.hh"
#include "debug/DtuCpuReq.hh"
#include "debug/DtuXlate.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/msg_unit.hh"
#include "mem/dtu/mem_unit.hh"
#include "mem/dtu/xfer_unit.hh"
#include "mem/dtu/pt_unit.hh"
#include "mem/cache/cache.hh"
#include "sim/dtu_memory.hh"
#include "sim/system.hh"

static const char *cmdNames[] =
{
    "IDLE",
    "SEND",
    "REPLY",
    "READ",
    "WRITE",
    "FETCH_MSG",
    "ACK_MSG",
    "SLEEP",
    "CLEAR_IRQ",
    "PRINT",
};

static const char *extCmdNames[] =
{
    "IDLE",
    "WAKEUP_CORE",
    "INV_EP",
    "INV_PAGE",
    "INV_TLB",
    "RESET",
    "ACK_MSG",
};

// cmdId = 0 is reserved for "no command"
// cmdId = 1 is reserved for a dummy command (waiting for remote xfers)
uint64_t Dtu::nextCmdId = 2;

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    masterId(p->system->getMasterId(name())),
    system(p->system),
    regFile(*this, name() + ".regFile", p->num_endpoints, p->num_header),
    connector(p->connector),
    tlBuf(p->tlb_entries > 0 ? new DtuTlb(*this, p->tlb_entries) : NULL),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, p->block_size, p->buf_count, p->buf_size)),
    ptUnit(p->pt_walker ? new PtUnit(*this) : NULL),
    abortCommandEvent(*this),
    completeTranslateEvent(*this),
    sleepStart(0),
    cmdPkt(),
    cmdFinish(),
    cmdId(0),
    abortCmd(0),
    cmdXferBuf(0),
    xlates(),
    coreXlates(new CoreTranslation[p->buf_count + 1]()),
    coreXlateSlots(p->buf_count + 1),
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

    DTUMemory *sys = dynamic_cast<DTUMemory*>(system);
    if (sys)
    {
        NocAddr phys = sys->getPhys(0);
        memPe = phys.coreId;
        memOffset = phys.offset;
        memSize = sys->memSize;
        DPRINTF(Dtu, "Using memory range %p .. %p\n",
            memOffset, memOffset + memSize);

        regs().set(DtuReg::ROOT_PT, sys->getRootPt().getAddr());
    }
    // memory PEs don't participate in cache coherence
    else
        coherent = false;

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

    extCommands
        .init(sizeof(extCmdNames) / sizeof(extCmdNames[0]))
        .name(name() + ".extCommands")
        .desc("The executed external commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(extCmdNames) / sizeof(extCmdNames[0]); ++i)
        extCommands.subname(i, extCmdNames[i]);

    xlateReqs
        .name(name() + ".xlateReqs")
        .desc("Number of translate requests to the core");
    xlateDelays
        .name(name() + ".xlateDelays")
        .desc("Number of delayed translate requests to the core");
    xlateFails
        .name(name() + ".xlateFails")
        .desc("Number of failed translate requests to the core");

    if (tlb())
        tlb()->regStats();
    if (ptUnit)
        ptUnit->regStats();
    xferUnit->regStats();
    memUnit->regStats();
    msgUnit->regStats();
}

bool
Dtu::isMemPE(unsigned pe) const
{
    DTUMemory *sys = dynamic_cast<DTUMemory*>(system);
    return !sys || sys->hasMem(pe);
}

PacketPtr
Dtu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
{
    Request::Flags flags;

    auto req = new Request(paddr, size, flags, masterId);

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
    delete pkt->req;
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
            regs().set(CmdReg::OFFSET, msgUnit->fetchMessage(cmd.epid));
            finishCommand(Error::NONE);
            break;
        case Command::ACK_MSG:
            msgUnit->ackMessage(cmd.epid, cmd.arg);
            finishCommand(Error::NONE);
            break;
        case Command::SLEEP:
            if (!startSleep(cmd.arg))
                finishCommand(Error::NONE);
            break;
        case Command::CLEAR_IRQ:
            clearIrq();
            finishCommand(Error::NONE);
            break;
        case Command::PRINT:
        {
            const DataReg data = regs().getDataReg();
            printLine(data.addr, data.size);
            finishCommand(Error::NONE);
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

    if ((abortCmd & Command::ABORT_CMD) && cmd.opcode != Command::IDLE)
    {
        DPRINTF(DtuCmd, "Aborting command %s with EP=%u, flags=%#x (id=%llu)\n",
                cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid,
                cmd.flags, cmdId);
    }

    if (abortCmd & Command::ABORT_VPE)
    {
        DPRINTF(DtuCmd, "Disabling communication and aborting PFs\n");

        RegFile::reg_t val = regs().get(DtuReg::FEATURES);
        val |= static_cast<RegFile::reg_t>(Features::COM_DISABLED);
        regs().set(DtuReg::FEATURES, val);

        if (ptUnit)
            ptUnit->abortAll();
    }

    cmdXferBuf = -1;
    uint types = 0;
    if (abortCmd & Command::ABORT_CMD)
        types |= XferUnit::ABORT_LOCAL;
    if (abortCmd & Command::ABORT_VPE)
    {
        types |= XferUnit::ABORT_REMOTE;
        cmdXferBuf = -1;
    }
    xferUnit->abortTransfers(types);

    regs().set(CmdReg::ABORT, cmdXferBuf);

    // if no command was running, let the SW wait until the currently received
    // messages are finished
    Error err;
    if (cmd.opcode == Command::IDLE)
    {
        if (abortCmd & Command::ABORT_VPE)
        {
            regs().set(CmdReg::COMMAND, Command::PRINT);
            cmdId = 1;
        }
        // all done
        else
            abortCmd = 0;
        err = Error::NONE;
    }
    // reads/writes are aborted immediately
    else if(cmd.opcode == Command::READ || cmd.opcode == Command::WRITE)
    {
        cmdId = 0;
        err = Error::ABORT;
    }

    if (cmd.opcode == Command::IDLE ||
        cmd.opcode == Command::READ ||
        cmd.opcode == Command::WRITE)
        scheduleFinishOp(Cycles(1), err);
}

void
Dtu::scheduleFinishOp(Cycles delay, Error error)
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
Dtu::finishCommand(Error error)
{
    Command::Bits cmd = getCommand();

    cmdFinish = NULL;

    if (abortCmd)
    {
        // try to abort everything
        uint types = 0;
        if (abortCmd & Command::ABORT_CMD)
            types |= XferUnit::ABORT_LOCAL;
        if (abortCmd & Command::ABORT_VPE)
            types |= XferUnit::ABORT_REMOTE;
        if (!xferUnit->abortTransfers(types))
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
    else if (error == Error::NONE && !abortCmd &&
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

    extCommands[static_cast<size_t>(cmd.opcode)]++;

    Cycles delay(1);

    Error result = Error::NONE;

    switch (cmd.opcode)
    {
        case ExternCommand::IDLE:
            break;
        case ExternCommand::WAKEUP_CORE:
            if (cmd.arg != 0)
                connector->reset(cmd.arg);
            wakeupCore();
            break;
        case ExternCommand::INV_EP:
            if (!regs().invalidate(cmd.arg))
                result = Error::MISS_CREDITS;
            break;
        case ExternCommand::INV_PAGE:
            if (tlb())
                tlb()->remove(cmd.arg);
            break;
        case ExternCommand::INV_TLB:
            if (tlb())
                tlb()->clear();
            break;
        case ExternCommand::RESET:
            delay += reset();
            break;
        case ExternCommand::ACK_MSG:
        {
            unsigned epid = cmd.arg & ((1 << 8) - 1);
            msgUnit->ackMessage(epid, cmd.arg >> 8);
            break;
        }
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (pkt)
    {
        auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);
        assert(senderState != nullptr);
        senderState->result = result;
        schedNocResponse(pkt, clockEdge(delay));
    }

    DPRINTF(DtuCmd, "Executing extern command %s with arg=%p -> %u\n",
            extCmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg,
            (unsigned)result);

    // set external command back to IDLE
    regFile.set(DtuReg::EXT_CMD,
        static_cast<RegFile::reg_t>(ExternCommand::IDLE));
}

bool
Dtu::startSleep(uint64_t cycles)
{
    if ((regFile.get(DtuReg::MSG_CNT) & 0xFFFF) > 0)
        return false;
    if (regFile.get(ReqReg::EXT_REQ) != 0)
        return false;
    if (regFile.get(ReqReg::XLATE_REQ) != 0)
        return false;

    // remember when we started
    sleepStart = curCycle();

    DPRINTF(Dtu, "Suspending CU\n");
    connector->suspend();

    if (cycles)
        scheduleFinishOp(Cycles(cycles));
    return true;
}

void
Dtu::stopSleep()
{
    if (sleepStart != 0)
    {
        // increase idle time in status register
        RegFile::reg_t counter = regFile.get(DtuReg::IDLE_TIME);
        counter += curCycle() - sleepStart;
        regFile.set(DtuReg::IDLE_TIME, counter);

        sleepStart = Cycles(0);

        DPRINTF(Dtu, "Waking up CU\n");
        connector->wakeup();
    }
}

void
Dtu::wakeupCore()
{
    if (getCommand().opcode == Command::SLEEP)
        scheduleFinishOp(Cycles(1));
    else
        connector->wakeup();
}

Cycles
Dtu::reset()
{
    Cycles delay(0);
    if(!coherent)
    {
        for (auto &c : caches)
        {
            // no writeback necessary
            c->memWriteback();
            c->memInvalidate();
            delay += Cycles(c->getBlockCount() / cacheBlocksPerCycle);
        }
    }

    // hard-abort everything
    xferUnit->abortTransfers(
        XferUnit::ABORT_LOCAL | XferUnit::ABORT_REMOTE | XferUnit::ABORT_MSGS);

    if(ptUnit)
        ptUnit->abortAll();

    if (tlb())
        tlb()->clear();

    regs().resetHeader();

    connector->reset(0);

    // since we did a reset & suspend, restart the sleep
    sleepStart = curCycle();

    resets++;
    return delay;
}

void
Dtu::setIrq()
{
    RegFile::reg_t val = regs().get(DtuReg::FEATURES);
    if (getCommand().opcode == Command::SLEEP)
        val |= static_cast<RegFile::reg_t>(Features::IRQ_WAKEUP);
    else
        val &= ~static_cast<RegFile::reg_t>(Features::IRQ_WAKEUP);
    regs().set(DtuReg::FEATURES, val);

    wakeupCore();

    connector->setIrq();

    irqInjects++;
}

void
Dtu::clearIrq()
{
    connector->clearIrq();
}

void
Dtu::printLine(Addr addr, Addr size)
{
    char buffer[256];
    size_t pos = 0;
    size_t rem = std::min(sizeof(buffer) - 1, size);
    while (rem > 0)
    {
        size_t off = addr & (system->cacheLineSize() - 1);
        size_t amount = std::min(rem, system->cacheLineSize() - off);


        NocAddr phys(addr);
        if(tlb() && ptUnit)
        {
            DtuTlb::Result res = tlb()->lookup(addr, DtuTlb::READ, &phys);
            assert(res != DtuTlb::PAGEFAULT);
            assert(res != DtuTlb::NOMAP);

            if(res == DtuTlb::MISS)
            {
                int xlate = ptUnit->translateFunctional(addr, DtuTlb::READ, &phys);
                assert(xlate == 1);
            }
        }

        auto pkt = generateRequest(phys.getAddr(), amount, MemCmd::ReadReq);
        dcacheMasterPort.sendFunctional(pkt);

        for(size_t i = 0; i < amount; ++i)
        {
            // ignore newlines. we have our own at the end
            if(pkt->getPtr<char>()[i] != '\n')
                buffer[pos++] = pkt->getPtr<char>()[i];
        }

        freeRequest(pkt);

        rem -= amount;
        addr += amount;
    }
    buffer[pos] = '\0';

    DPRINTF(Dtu, "PRINT: %s\n", buffer);
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
Dtu::physToNoc(Addr phys) const
{
    Addr noc = phys;
    if (!ptUnit)
        noc = (noc & ~0x0000FF0000000000ULL) | ((noc & 0x0000FF0000000000ULL) << 16);
    return noc;
}

Addr
Dtu::nocToPhys(Addr noc) const
{
    Addr phys = noc;
    if (!ptUnit)
        phys = (phys & ~0xFF00000000000000ULL) | ((phys & 0xFF00000000000000ULL) >> 16);
    return phys;
}

void
Dtu::startTransfer(void *event, Cycles delay)
{
    auto ev = reinterpret_cast<XferUnit::TransferEvent*>(event);
    xferUnit->startTransfer(ev, delay);
}

void
Dtu::startTranslate(size_t id,
                    Addr virt,
                    uint access,
                    PtUnit::Translation *trans)
{
    if (ptUnit)
        ptUnit->startTranslate(virt, access, trans);
    else
    {
        if (coreXlates[id].trans)
            return;

        coreXlates[id].trans = trans;
        coreXlates[id].virt = virt;
        coreXlates[id].access = access;
        coreXlates[id].ongoing = false;

        DPRINTF(DtuXlate, "Translation[%lu] = %p:%#x\n",
            id, virt, access);
        xlateReqs++;

        if(regs().get(ReqReg::XLATE_REQ) == 0)
        {
            const Addr mask = DtuTlb::PAGE_MASK;
            const Addr virtPage = coreXlates[id].virt & ~mask;
            regs().set(ReqReg::XLATE_REQ,
                virtPage | coreXlates[id].access | (id << 5));
            coreXlates[id].ongoing = true;

            DPRINTF(DtuXlate, "Translation[%lu] started\n", id);
            setIrq();
        }
        else
            xlateDelays++;
    }
}

void
Dtu::completeTranslate()
{
    RegFile::reg_t resp = regs().get(ReqReg::XLATE_RESP);
    if (resp)
    {
        size_t id = (resp >> 5) & 0x7;
        Addr mask = (resp & DtuTlb::LARGE) ? DtuTlb::LPAGE_MASK
                                           : DtuTlb::PAGE_MASK;
        assert(coreXlates[id].trans);
        assert(coreXlates[id].ongoing);
        DPRINTF(DtuXlate, "Translation[%lu] done\n", id);

        NocAddr phys((resp & ~mask) | (coreXlates[id].virt & mask));
        uint flags = resp & (DtuTlb::IRWX | DtuTlb::LARGE);
        if (flags == 0)
        {
            coreXlates[id].trans->finished(false, phys);
            xlateFails++;
        }
        else
        {
            tlb()->insert(coreXlates[id].virt, phys, flags);
            coreXlates[id].trans->finished(true, phys);
        }

        coreXlates[id].trans = nullptr;
        regs().set(ReqReg::XLATE_RESP, 0);
    }

    if (regs().get(ReqReg::XLATE_REQ) == 0)
    {
        for (size_t id = 0; id < coreXlateSlots; ++id)
        {
            if (coreXlates[id].trans && !coreXlates[id].ongoing)
            {
                const Addr mask = DtuTlb::PAGE_MASK;
                const Addr virtPage = coreXlates[id].virt & ~mask;
                regs().set(ReqReg::XLATE_REQ,
                    virtPage | coreXlates[id].access | (id << 5));
                coreXlates[id].ongoing = true;
                DPRINTF(DtuXlate, "Translation[%lu] started\n", id);

                setIrq();
                break;
            }
        }
    }
}

void
Dtu::abortTranslate(size_t id, PtUnit::Translation *trans)
{
    if (ptUnit)
        ptUnit->abortTranslate(trans);
    else
    {
        coreXlates[id].trans = NULL;
        coreXlates[id].ongoing = false;
        cmdXferBuf = id;
    }
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
        // as these target memory PEs, there can't be any error
        assert(senderState->result == Error::NONE);

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

        if (!ptUnit)
        {
            // translate NoC address to physical address
            Addr phys = nocToPhys(pkt->getAddr());
            pkt->setAddr(phys);
            pkt->req->setPaddr(phys);
        }

        sendCacheMemResponse(pkt, true);
    }
    else if (senderState->packetType == NocPacketType::PAGEFAULT)
    {
        if (senderState->result != Error::NONE)
            ptUnit->sendingPfFailed(pkt, static_cast<int>(senderState->result));
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

    switch(senderState->type)
    {
        case MemReqType::TRANSFER:
            xferUnit->recvMemResponse(senderState->data, pkt);
            break;

        case MemReqType::TRANSLATION:
            ptUnit->recvFromMem(senderState->data, pkt);
            break;
    }

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

    Error res = Error::NONE;

    switch (senderState->packetType)
    {
        case NocPacketType::MESSAGE:
        case NocPacketType::PAGEFAULT:
        {
            nocMsgRecvs++;
            uint flags = senderState->flags;
            res = msgUnit->recvFromNoc(pkt, senderState->vpeId, flags);
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
            res = memUnit->recvFromNoc(pkt, senderState->vpeId, flags);
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
Dtu::handleCpuRequest(PacketPtr pkt,
                      DtuSlavePort &sport,
                      DtuMasterPort &mport,
                      bool icache,
                      bool functional)
{
    bool res = true;
    bool delayed = false;
    Addr virt = pkt->getAddr();

    DPRINTF(DtuCpuReq, "%s access for %#lx: start\n",
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

        MemTranslation *trans = nullptr;
        int tres = 1;
        // CPU requests are only translated if we have a PT unit
        if (ptUnit)
        {
            trans = new MemTranslation(*this, sport, mport, pkt);
            tres = translate(trans, pkt, icache, functional);
        }

        if (tres == 1)
        {
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
            delete trans;
        }
        else if (tres == -1)
        {
            res = false;
            delete trans;
        }
        else
        {
            delayed = true;
            // remember that a translation is going on for that page.
            // this way, subsequent requests to that page will be enqueued and
            // thus sent to the cache in order.
            tlb()->start_translate(virt);
            xlates.push_back(trans);
        }
    }

    if (!delayed)
    {
        DPRINTF(DtuCpuReq, "%s access for %#lx: finished\n",
            pkt->cmdString(), virt);
    }

    return res;
}

void
Dtu::completeCpuRequests()
{
    while (!xlates.empty())
    {
        MemTranslation *xlt = xlates.front();
        if (!xlt->complete)
            break;

        DPRINTF(DtuCpuReq, "%s access for %#lx: finished (success=%d, phys=%#lx)\n",
            xlt->pkt->cmdString(), xlt->pkt->getAddr(),
            xlt->success, xlt->phys.getAddr());

        // translation is done. if no other translation is in progress for that
        // page, requests can hit the TLB again and can be sent directly to the
        // cache.
        tlb()->finish_translate(xlt->pkt->getAddr());

        if (!xlt->success)
            sendDummyResponse(xlt->sport, xlt->pkt, false);
        else
        {
            xlt->pkt->setAddr(xlt->phys.getAddr());
            xlt->pkt->req->setPaddr(xlt->phys.getAddr());

            xlt->mport.schedTimingReq(xlt->pkt, curTick());
        }

        xlates.pop_front();
        delete xlt;
    }
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
    Addr nocAddr = physAddr;
    if (!ptUnit)
    {
        nocAddr = physToNoc(physAddr);
        pkt->setAddr(nocAddr);
    }

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
                   INVALID_VPE_ID,
                   NocFlags::NONE,
                   Cycles(1),
                   functional);

    extMemReqs++;

    if (functional)
        pkt->setAddr(physAddr);

    return true;
}

int
Dtu::translate(PtUnit::Translation *trans,
               PacketPtr pkt,
               bool icache,
               bool functional)
{
    assert(ptUnit);

    uint access = DtuTlb::INTERN;
    if (icache)
        access |= DtuTlb::EXEC;
    if (pkt->isRead())
        access |= DtuTlb::READ;
    if (pkt->isWrite())
        access |= DtuTlb::WRITE;

    NocAddr phys;
    DtuTlb::Result res = tlb()->lookup(pkt->getAddr(), access, &phys);
    switch(res)
    {
        case DtuTlb::HIT:
            pkt->setAddr(phys.getAddr());
            pkt->req->setPaddr(phys.getAddr());
            return 1;

        case DtuTlb::NOMAP:
            // don't cause a pagefault again in this case
            return -1;

        default:
            if (functional)
            {
                int xlres = 0;
                if (res == DtuTlb::MISS)
                {
                    NocAddr phys;
                    xlres = ptUnit->translateFunctional(pkt->getAddr(),
                                                        access,
                                                        &phys);
                    if (xlres)
                    {
                        pkt->setAddr(phys.getAddr());
                        pkt->req->setPaddr(phys.getAddr());
                    }
                }

                return !xlres ? -1 : 1;
            }

            ptUnit->startTranslate(pkt->getAddr(), access, trans);
            return 0;
    }
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

        if (~result & RegFile::WROTE_EXT_CMD)
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
                schedule(completeTranslateEvent, when);
            else if (result & RegFile::WROTE_EXT_REQ)
                setIrq();
        }
        else
            schedule(new ExecExternCmdEvent(*this, pkt), when);
    }
    else
    {
        if (result & RegFile::WROTE_CMD)
            executeCommand(NULL);
        if (result & RegFile::WROTE_EXT_CMD)
            executeExternCommand(NULL);
        if (result & RegFile::WROTE_ABORT)
            abortCommand();
        if (result & RegFile::WROTE_XLATE)
            completeTranslate();
        if (result & RegFile::WROTE_EXT_REQ)
            setIrq();
    }
}

void
Dtu::MemTranslation::finished(bool _success, const NocAddr &_phys)
{
    complete = true;
    success = _success;
    phys = _phys;

    dtu.completeCpuRequests();
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
