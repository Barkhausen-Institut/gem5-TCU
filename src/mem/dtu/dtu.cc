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
#include "arch/x86/interrupts.hh"
#include "debug/Dtu.hh"
#include "debug/DtuBuf.hh"
#include "debug/DtuCmd.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuSysCalls.hh"
#include "debug/DtuPower.hh"
#include "debug/DtuMem.hh"
#include "debug/DtuTlb.hh"
#include "cpu/simple/base.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/msg_unit.hh"
#include "mem/dtu/mem_unit.hh"
#include "mem/dtu/xfer_unit.hh"
#include "mem/dtu/pt_unit.hh"
#include "mem/page_table.hh"
#include "sim/system.hh"
#include "sim/process.hh"

static const char *cmdNames[] =
{
    "IDLE",
    "SEND",
    "REPLY",
    "READ",
    "WRITE",
    "INC_READ_PTR",
};

static const char *extCmdNames[] =
{
    "WAKEUP_CORE",
    "INV_PAGE",
    "INJECT_IRQ",
};

Dtu::Dtu(DtuParams* p)
  : BaseDtu(p),
    masterId(p->system->getMasterId(name())),
    system(p->system),
    regFile(name() + ".regFile", p->num_endpoints),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, p->block_size, p->buf_count, p->buf_size)),
    ptUnit(p->tlb_entries > 0 ? new PtUnit(*this) : NULL),
    executeCommandEvent(*this),
    executeExternCommandEvent(*this),
    cmdInProgress(false),
    tlb(p->tlb_entries > 0 ? new DtuTlb(p->tlb_entries) : NULL),
    memPe(),
    memOffset(),
    memSize(),
    atomicMode(p->system->isAtomicMode()),
    numEndpoints(p->num_endpoints),
    maxNocPacketSize(p->max_noc_packet_size),
    numCmdEpidBits(p->num_cmd_epid_bits),
    blockSize(p->block_size),
    bufCount(p->buf_count),
    bufSize(p->buf_size),
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
        memSize = sys->memSize;
        regs().set(DtuReg::ROOT_PT, sys->getRootPt().getAddr());
        regs().set(DtuReg::VPE_ID, INVALID_VPE_ID);
    }
}

Dtu::~Dtu()
{
    delete xferUnit;
    delete memUnit;
    delete msgUnit;
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
    assert(numCmdEpidBits + numCmdOpcodeBits <= sizeof(RegFile::reg_t) * 8);

    using reg_t = RegFile::reg_t;

    /*
     *   COMMAND                     0
     * |-----------------------------|
     * |  error  |  epid  |  opcode  |
     * |-----------------------------|
     */
    reg_t opcodeMask = ((reg_t)1 << numCmdOpcodeBits) - 1;
    reg_t epidMask   = (((reg_t)1 << numCmdEpidBits) - 1) << numCmdOpcodeBits;

    auto reg = regFile.get(CmdReg::COMMAND);

    Command cmd;

    unsigned bits = numCmdOpcodeBits + numCmdEpidBits;
    cmd.error = static_cast<Error>(reg >> bits);

    cmd.opcode = static_cast<Command::Opcode>(reg & opcodeMask);

    cmd.epId = (reg & epidMask) >> numCmdOpcodeBits;

    return cmd;
}

void
Dtu::executeCommand()
{
    Command cmd = getCommand();
    if (cmd.opcode == Command::IDLE)
        return;

    assert(!cmdInProgress);
    assert(cmd.epId < numEndpoints);

    cmdInProgress = true;

    DPRINTF(DtuCmd, "Starting command %s with EP%d\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epId);

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
        msgUnit->incrementReadPtr(cmd.epId);
        finishCommand(NONE);
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

    DPRINTF(DtuCmd, "Finished command %s with EP%d -> %u\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epId, error);

    // let the SW know that the command is finished
    unsigned bits = numCmdOpcodeBits + numCmdEpidBits;
    regFile.set(CmdReg::COMMAND, error << bits);

    cmdInProgress = false;
}

Dtu::ExternCommand
Dtu::getExternCommand()
{
    auto reg = regFile.get(DtuReg::EXT_CMD);

    ExternCommand cmd;
    cmd.opcode = static_cast<ExternCommand::Opcode>(reg & 0x3);
    cmd.arg = reg >> 2;
    return cmd;
}

void
Dtu::executeExternCommand()
{
    ExternCommand cmd = getExternCommand();

    DPRINTF(DtuCmd, "Executing extern command %s with arg=%p\n",
            extCmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg);

    switch (cmd.opcode)
    {
    case ExternCommand::WAKEUP_CORE:
        wakeupCore();
        break;
    case ExternCommand::INV_PAGE:
        if (tlb)
            tlb->remove(cmd.arg);
        break;
    case ExternCommand::INJECT_IRQ:
        injectIRQ(cmd.arg);
        break;
    default:
        // TODO error handling
        panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }
}

void
Dtu::wakeupCore()
{
    if (system->threadContexts.size() == 0)
        return;

    if (system->threadContexts[0]->status() == ThreadContext::Suspended)
    {
        DPRINTF(DtuPower, "Waking up core\n");
        system->threadContexts[0]->activate();
    }
}

void
Dtu::updateSuspendablePin()
{
    if (system->threadContexts.size() == 0)
        return;

    bool pendingMsgs = regFile.get(DtuReg::MSG_CNT) > 0;
    bool hadPending = system->threadContexts[0]->getCpuPtr()->_denySuspend;
    system->threadContexts[0]->getCpuPtr()->_denySuspend = pendingMsgs;
    if (hadPending && !pendingMsgs)
        DPRINTF(DtuPower, "Core can be suspended\n");
}

void
Dtu::injectIRQ(int vector)
{
    const int APIC_ID = 0;

    X86ISA::TriggerIntMessage message = 0;
    message.deliveryMode = X86ISA::DeliveryMode::ExtInt;
    message.destination = APIC_ID;
    message.destMode = 0;   // physical
    message.trigger = 0;    // edge
    message.level = 0;      // unused?
    message.vector = vector;

    PacketPtr pkt = X86ISA::buildIntRequest(APIC_ID, message);
    sendIRQRequest(pkt);
}

void
Dtu::sendMemRequest(PacketPtr pkt,
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
                    Cycles delay,
                    bool functional)
{
    auto senderState = new NocSenderState();
    senderState->packetType = type;
    senderState->result = NONE;

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
                   MessageHeader* header,
                   Cycles delay,
                   uint flags)
{
    xferUnit->startTransfer(type,
                            targetAddr,
                            sourceAddr,
                            size,
                            pkt,
                            header,
                            delay,
                            flags);
}

void
Dtu::startTranslate(Addr virt,
                    uint access,
                    PtUnit::Translation *trans,
                    bool pf)
{
    ptUnit->startTranslate(virt, access, trans, pf);
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
            senderState->result);

        if (dynamic_cast<InitSenderState*>(pkt->senderState))
        {
            // undo the change from handleCacheMemRequest
            pkt->setAddr(phys.offset - memOffset);
            pkt->req->setPaddr(phys.offset - memOffset);
            pkt->popSenderState();
        }

        if (senderState->result != NONE)
        {
            uint access = DtuTlb::INTERN | DtuTlb::GONE;
            VPEGoneTranslation *trans = new VPEGoneTranslation(*this, pkt);
            ptUnit->startTranslate(pkt->getAddr(), access, trans, true);
        }
        else
            sendCacheMemResponse(pkt, true);
    }
    else if (senderState->packetType == NocPacketType::PAGEFAULT)
    {
        if (senderState->result != NONE)
            ptUnit->sendingPfFailed(pkt, senderState->result);
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

    delete senderState;
    freeRequest(pkt);
}

void
Dtu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());

    auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);

    Error res = NONE;

    switch (senderState->packetType)
    {
    case NocPacketType::MESSAGE:
    case NocPacketType::PAGEFAULT:
        res = msgUnit->recvFromNoc(pkt);
        break;
    case NocPacketType::READ_REQ:
    case NocPacketType::WRITE_REQ:
    case NocPacketType::CACHE_MEM_REQ:
        res = memUnit->recvFromNoc(pkt);
        break;
    case NocPacketType::CACHE_MEM_REQ_FUNC:
        memUnit->recvFunctionalFromNoc(pkt);
        break;
    default:
        panic("Unexpected NocPacketType\n");
    }

    senderState->result = res;
}

void
Dtu::handleCpuRequest(PacketPtr pkt)
{
    forwardRequestToRegFile(pkt, true);
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
        phys = NocAddr(memPe, 0, memOffset + phys.offset);
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
    sendNocRequest(type, pkt, Cycles(1), functional);

    if (functional)
        pkt->setAddr(old);

    return true;
}

bool
Dtu::translate(PtUnit::Translation *trans,
               PacketPtr pkt,
               bool icache,
               bool functional)
{
    if (!tlb)
        return true;

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
    DtuTlb::Result res = tlb->lookup(pkt->getAddr(), access, &phys);
    switch(res)
    {
        case DtuTlb::HIT:
            DPRINTF(DtuTlb, "Translated %s access for %p -> %p\n",
                    icache ? "exec" : (pkt->isRead() ? "read" : "write"),
                    pkt->getAddr(), phys.getAddr());

            pkt->setAddr(phys.getAddr());
            pkt->req->setPaddr(phys.getAddr());
            break;

        case DtuTlb::MISS:
        case DtuTlb::PAGEFAULT:
        {
            bool pf = res == DtuTlb::PAGEFAULT;
            DPRINTF(DtuTlb, "%s for %s access to %p\n",
                    pf ? "Pagefault" : "TLB-miss",
                    icache ? "exec" : (pkt->isRead() ? "read" : "write"),
                    pkt->getAddr());

            if (functional)
            {
                assert(!pf);
                NocAddr phys;
                bool res = ptUnit->translateFunctional(pkt->getAddr(),
                                                       access,
                                                       &phys);
                // TODO handle errors here
                assert(res);
                pkt->setAddr(phys.getAddr());
                pkt->req->setPaddr(phys.getAddr());
                return true;
            }

            ptUnit->startTranslate(pkt->getAddr(), access, trans, pf);
        }
        return false;
    }

    return true;
}

void
Dtu::forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest)
{
    Addr oldAddr = pkt->getAddr();

    // Strip the base address to handle requests based on the reg. addr. only.
    pkt->setAddr(oldAddr - regFileBaseAddr);

    RegFile::Result result = regFile.handleRequest(pkt, isCpuRequest);

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

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        if (isCpuRequest)
            schedCpuResponse(pkt, when);
        else
        {
            schedNocRequestFinished(clockEdge(Cycles(1)));
            schedNocResponse(pkt, when);
        }

        if (result & RegFile::WROTE_CMD)
            schedule(executeCommandEvent, when);
        if (result & RegFile::WROTE_EXT_CMD)
            schedule(executeExternCommandEvent, when);
    }
    else
    {
        if (result & RegFile::WROTE_CMD)
            executeCommand();
        if (result & RegFile::WROTE_EXT_CMD)
            executeExternCommand();
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
