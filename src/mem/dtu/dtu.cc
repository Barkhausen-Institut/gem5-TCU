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
    "FETCH_MSG",
    "ACK_MSG",
    "SLEEP",
    "DEBUG_MSG",
};

static const char *extCmdNames[] =
{
    "IDLE",
    "WAKEUP_CORE",
    "INV_PAGE",
    "INV_TLB",
    "INJECT_IRQ",
    "RESET",
};

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    masterId(p->system->getMasterId(name())),
    system(p->system),
    regFile(*this, name() + ".regFile", p->num_endpoints),
    connector(p->connector),
    tlBuf(p->tlb_entries > 0 ? new DtuTlb(*this, p->tlb_entries) : NULL),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, p->block_size, p->buf_count, p->buf_size)),
    ptUnit(p->tlb_entries > 0 ? new PtUnit(*this) : NULL),
    abortCommandEvent(*this),
    sleepStart(0),
    cmdPkt(),
    cmdFinish(),
    cmdInProgress(false),
    abortInProgress(false),
    irqPending(false),
    cmdDest(-1),
    memPe(),
    memOffset(),
    atomicMode(p->system->isAtomicMode()),
    numEndpoints(p->num_endpoints),
    maxNocPacketSize(p->max_noc_packet_size),
    numCmdEpBits(8),
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

    DTUMemory *sys = dynamic_cast<DTUMemory*>(system);
    if (sys)
    {
        NocAddr phys = sys->getPhys(0);
        memPe = phys.coreId;
        memOffset = phys.offset;
        DPRINTF(Dtu, "Using memory range %p .. %p\n",
            memOffset, memOffset + sys->memSize);

        regs().set(DtuReg::ROOT_PT, phys.getAddr());
    }
    // memory PEs don't participate in cache coherence
    else
        coherent = false;

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

Dtu::Command
Dtu::getCommand()
{
    assert(numCmdEpBits + numCmdFlagsBits + numCmdOpcodeBits <=
        sizeof(RegFile::reg_t) * 8);

    using reg_t = RegFile::reg_t;

    /*
     *                                          0
     * |----------------------------------------|
     * |  error  |   flags  |  epid  |  opcode  |
     * |----------------------------------------|
     */
    reg_t opcodeMask = ((reg_t)1 << numCmdOpcodeBits) - 1;
    reg_t argMask = ((reg_t)1 << numCmdEpBits) - 1;
    reg_t flagMask = ((reg_t)1 << numCmdFlagsBits) - 1;

    auto reg = regFile.get(CmdReg::COMMAND);

    Command cmd;

    unsigned bits = numCmdOpcodeBits + numCmdEpBits + numCmdFlagsBits;
    cmd.error  = static_cast<Error>(reg >> bits);

    cmd.opcode = static_cast<Command::Opcode>(reg & opcodeMask);

    cmd.epid   = (reg >> numCmdOpcodeBits) & argMask;

    cmd.flags  = (reg >> (numCmdEpBits + numCmdOpcodeBits)) & flagMask;

    return cmd;
}

void
Dtu::executeCommand(PacketPtr pkt)
{
    Command cmd = getCommand();
    if (cmd.opcode == Command::IDLE)
    {
        if (pkt)
            schedCpuResponse(pkt, clockEdge(Cycles(1)));
        return;
    }

    assert(!cmdInProgress);

    cmdPkt = pkt;
    cmdInProgress = true;
    commands[static_cast<size_t>(cmd.opcode)]++;

    if(cmd.opcode != Command::DEBUG_MSG)
    {
        assert(cmd.epid < numEndpoints);
        DPRINTF(DtuCmd, "Starting command %s with EP=%u, flags=%#x\n",
                cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid, cmd.flags);
    }

    // if the VPE id is invalid and we are not privileged, commands cannot
    // be executed
    if (regs().get(DtuReg::VPE_ID) == INVALID_VPE_ID &&
        !(regs().get(DtuReg::FEATURES) & static_cast<RegFile::reg_t>(Features::PRIV)))
    {
        scheduleFinishOp(Cycles(1), Error::VPEID_INVALID);
        return;
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
    case Command::FETCH_MSG:
        regs().set(CmdReg::OFFSET, msgUnit->fetchMessage(cmd.epid));
        finishCommand(Error::NONE);
        break;
    case Command::ACK_MSG:
        msgUnit->ackMessage(cmd.epid);
        finishCommand(Error::NONE);
        break;
    case Command::SLEEP:
        if (!startSleep())
            finishCommand(Error::NONE);
        break;
    case Command::DEBUG_MSG:
        DPRINTF(Dtu, "DEBUG %#x\n", regs().get(CmdReg::OFFSET));
        finishCommand(Error::NONE);
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
    Command cmd = getCommand();
    RegFile::reg_t abort = regs().get(CmdReg::ABORT);

    if ((abort & Command::ABORT_CMD) && cmd.opcode != Command::IDLE)
    {
        DPRINTF(DtuCmd, "Aborting command %s with EP=%u, flags=%#x\n",
                cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid, cmd.flags);

        // local transfers are only issued by commands, i.e., the current one
        size_t count = xferUnit->abortTransfers(XferUnit::ABORT_LOCAL, -1);
        assert(count <= 1);

        // if there was none and we already sent the request to the Noc,
        // send an abort request to the core we sent to
        if (count == 0 && cmdDest != -1)
        {
            assert(!abortInProgress);
            DPRINTF(DtuCmd, "Sending abort request to PE%2d\n", cmdDest);

            abortInProgress = true;
            Addr nocAddr = NocAddr(cmdDest, 0).getAddr();
            auto pkt = generateRequest(nocAddr, 0, MemCmd::WriteReq);

            sendNocRequest(NocPacketType::ABORT,
                           pkt,
                           INVALID_VPE_ID,  // not relevant here
                           0,
                           Cycles(1));
        }
    }

    if (abort & Command::ABORT_VPE)
    {
        DPRINTF(DtuCmd, "Resetting VPE id and aborting PFs\n");
        regs().set(DtuReg::VPE_ID, INVALID_VPE_ID);
        if (ptUnit)
            ptUnit->abortAll();
    }

    regs().set(CmdReg::ABORT, 0);

    // see comment below
    irqPending = false;
}

void
Dtu::scheduleFinishOp(Cycles delay, Error error)
{
    if (cmdInProgress)
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
    Command cmd = getCommand();

    assert(cmdInProgress);

    // if the command succeeded, or we still have a valid VPE id, pay the
    // credits or disable replies. the latter is required to prevent that a
    // malicious app causes NoC traffic all the time by aborting commands
    if (error == Error::NONE || regs().get(DtuReg::VPE_ID) != INVALID_VPE_ID)
    {
        if (cmd.opcode == Command::SEND)
            msgUnit->payCredits(cmd.epid);
        else if (cmd.opcode == Command::REPLY)
            msgUnit->disableReplies();
    }

    if (abortInProgress)
    {
        assert(cmdDest != -1);

        // for the reading case, we might have remembered to abort the next
        // local transfer
        xferUnit->abortTransfers(XferUnit::ABORT_ABORT, -1);

        // for writing, if there was no error, the command already completed
        // and we have to abort our abort request at the other DTU.
        // for reading, we always sent the abort abort
        // TODO we could optimize that by distinguishing local and remote aborts
        if (cmd.opcode == Command::READ || error == Error::NONE)
        {
            // otherwise, abort it at the other DTU
            DPRINTF(DtuCmd, "Sending abort abort request to PE%2d\n", cmdDest);

            Addr nocAddr = NocAddr(cmdDest, 0).getAddr();
            auto pkt = generateRequest(nocAddr, 0, MemCmd::WriteReq);

            sendNocRequest(NocPacketType::ABORT_ABORT,
                           pkt,
                           INVALID_VPE_ID,  // not relevant here
                           0,
                           Cycles(1));

            // TODO actually, we should finish the command if we received the
            // response for the ABORT_ABORT
        }

        abortInProgress = false;
    }

    if (cmd.opcode == Command::SLEEP)
        stopSleep();

    if (cmd.opcode != Command::DEBUG_MSG)
    {
        DPRINTF(DtuCmd, "Finished command %s with EP=%u, flags=%#x -> %u\n",
                cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid, cmd.flags,
                static_cast<uint>(error));
    }

    // let the SW know that the command is finished
    unsigned bits = numCmdOpcodeBits + numCmdFlagsBits + numCmdEpBits;
    regFile.set(CmdReg::COMMAND, static_cast<RegFile::reg_t>(error) << bits);

    if (cmdPkt)
        schedCpuResponse(cmdPkt, clockEdge(Cycles(1)));

    cmdPkt = NULL;
    cmdFinish = NULL;
    cmdInProgress = false;
    cmdDest = -1;
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
    case ExternCommand::IDLE:
        break;
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
    case ExternCommand::INJECT_IRQ:
        wakeupCore();
        injectIRQ(cmd.arg);
        break;
    case ExternCommand::RESET:
        delay += reset(cmd.arg);
        break;
    default:
        // TODO error handling
        panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (pkt)
        schedNocResponse(pkt, clockEdge(delay));

    // set external command back to IDLE
    regFile.set(DtuReg::EXT_CMD,
        static_cast<RegFile::reg_t>(ExternCommand::IDLE));
}

bool
Dtu::startSleep()
{
    if ((regFile.get(DtuReg::MSG_CNT) & 0xFFFF) > 0)
        return false;

    // it might be that after an injected IRQ that the core continues to
    // execute some instructions. if one of them instructs the DTU to sleep
    // we suspend the core again and thus halt everything. thus, we prevent
    // that by disallowing sleeps between injectIRQ and abort.
    // TODO this is not a good solution
    if (irqPending)
        return false;

    uint64_t cycles = regs().get(CmdReg::OFFSET);

    // remember when we started
    sleepStart = curCycle();

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

        connector->wakeup();
    }
}

void
Dtu::wakeupCore()
{
    connector->wakeup();

    if (cmdInProgress && getCommand().opcode == Command::SLEEP)
        scheduleFinishOp(Cycles(1));
}

void
Dtu::suspend()
{
    if ((regFile.get(DtuReg::MSG_CNT) & 0xFFFF) == 0)
        connector->suspend();
}

Cycles
Dtu::reset(Addr addr)
{
    Cycles delay(0);
    if(!coherent)
    {
        if(l1Cache)
        {
            l1Cache->memWriteback();
            l1Cache->memInvalidate();
            delay += Cycles(l1Cache->getBlockCount() / cacheBlocksPerCycle);
        }
        if(l2Cache)
        {
            l2Cache->memWriteback();
            l2Cache->memInvalidate();
            delay += Cycles(l2Cache->getBlockCount() / cacheBlocksPerCycle);
        }
    }

    xferUnit->abortTransfers(XferUnit::ABORT_LOCAL, -1, true);
    xferUnit->abortTransfers(XferUnit::ABORT_REMOTE, -1, true);

    if(ptUnit)
        ptUnit->abortAll();

    if (tlb())
        tlb()->clear();

    connector->reset(addr);
    connector->suspend();

    // since we did a reset & suspend, restart the sleep
    sleepStart = curCycle();

    resets++;
    return delay;
}

void
Dtu::injectIRQ(int vector)
{
    connector->injectIrq(vector);

    irqPending = true;

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
    senderState->sender = coreId;
    senderState->vpeId = vpeId;
    senderState->flags = flags;

    // remember the destination for commands (needed for aborts)
    if (type == NocPacketType::READ_REQ ||
        type == NocPacketType::WRITE_REQ ||
        type == NocPacketType::MESSAGE)
    {
        cmdDest = NocAddr(pkt->getAddr()).coreId;
    }

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
Dtu::startTransfer(void *event, Cycles delay)
{
    auto ev = reinterpret_cast<XferUnit::TransferEvent*>(event);
    xferUnit->startTransfer(ev, delay);
}

void
Dtu::startTranslate(Addr virt,
                    uint access,
                    PtUnit::Translation *trans)
{
    ptUnit->startTranslate(virt, access, trans);
}

void
Dtu::abortTranslate(PtUnit::Translation *trans)
{
    ptUnit->abortTranslate(trans);
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
    else if (senderState->packetType == NocPacketType::ABORT ||
             senderState->packetType == NocPacketType::ABORT_ABORT)
    {
        // nothing to do
    }
    else if (senderState->packetType != NocPacketType::CACHE_MEM_REQ_FUNC)
    {
        if (pkt->isWrite())
            memUnit->writeComplete(getCommand(), pkt, senderState->result);
        else if (pkt->isRead())
            memUnit->readComplete(getCommand(), pkt, senderState->result);
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
                                  pkt->getSize());
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

    if (pkt->memInhibitAsserted())
    {
        DPRINTF(DtuPackets, "Ignoring inhibited packet\n");
        schedNocRequestFinished(clockEdge(Cycles(1)));
        return;
    }

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
    case NocPacketType::ABORT:
        xferUnit->abortTransfers(XferUnit::ABORT_REMOTE, senderState->sender);
        break;
    case NocPacketType::ABORT_ABORT:
        xferUnit->abortTransfers(XferUnit::ABORT_ABORT, senderState->sender);
        break;
    default:
        panic("Unexpected NocPacketType\n");
    }

    senderState->result = res;

    // in this case, we can respond immediately
    if (senderState->packetType == NocPacketType::ABORT ||
        senderState->packetType == NocPacketType::ABORT_ABORT)
        sendNocResponse(pkt);
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
                    pkt->cmdString(),
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
            pkt->setAddr(phys.getAddr());
            pkt->req->setPaddr(phys.getAddr());
            break;

        case DtuTlb::NOMAP:
        case DtuTlb::MISS:
        case DtuTlb::PAGEFAULT:
        {
            bool pf = res != DtuTlb::MISS;
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
