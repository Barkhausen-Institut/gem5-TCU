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

#include "debug/TcuCmd.hh"
#include "mem/tcu/cmds.hh"
#include "mem/tcu/mem_unit.hh"
#include "mem/tcu/msg_unit.hh"
#include "mem/tcu/tcu.hh"

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
};

static const char *privCmdNames[] =
{
    "IDLE",
    "INV_PAGE",
    "INV_TLB",
    "INS_TLB",
    "XCHG_VPE",
    "SET_TIMER",
    "ABORT_CMD",
    "FLUSH_CACHE",
};

static const char *extCmdNames[] =
{
    "IDLE",
    "INV_EP",
    "RESET",
};

const std::string
TcuCommands::CmdEvent::name() const
{
    return cmds.tcu.name();
}

TcuCommands::TcuCommands(Tcu &_tcu)
    : tcu(_tcu),
      cmdPkt(),
      privCmdPkt(),
      cmdFinish(),
      abort(),
      cmdIsRemote()
{
    static_assert(sizeof(cmdNames) / sizeof(cmdNames[0]) ==
        CmdCommand::SLEEP + 1, "cmdNames out of sync");
    static_assert(sizeof(privCmdNames) / sizeof(privCmdNames[0]) ==
        PrivCommand::FLUSH_CACHE + 1, "privCmdNames out of sync");
    static_assert(sizeof(extCmdNames) / sizeof(extCmdNames[0]) ==
        ExtCommand::RESET + 1, "extCmdNames out of sync");
}

const std::string
TcuCommands::name() const
{
    return tcu.name();
}

void
TcuCommands::regStats()
{
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

    extCommands
        .init(sizeof(extCmdNames) / sizeof(extCmdNames[0]))
        .name(name() + ".extCommands")
        .desc("The executed external commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(extCmdNames) / sizeof(extCmdNames[0]); ++i)
        extCommands.subname(i, extCmdNames[i]);
}

void
TcuCommands::startCommand(RegFile::Result written, PacketPtr pkt, Tick when)
{
    if (written & RegFile::WROTE_EXT_CMD)
        tcu.schedule(new ExecExtCmdEvent(*this, pkt), when);
    if (written & RegFile::WROTE_CMD)
        tcu.schedule(new ExecCmdEvent(*this, pkt), when);
    if (written & RegFile::WROTE_PRIV_CMD)
        tcu.schedule(new ExecPrivCmdEvent(*this, pkt), when);
}

void
TcuCommands::stopCommand()
{
    if (cmdPkt)
    {
        tcu.stopSleep();
        tcu.schedCpuResponse(cmdPkt, tcu.clockEdge(Cycles(1)));
        cmdPkt = NULL;
    }
}

void
TcuCommands::executeCommand(PacketPtr pkt)
{
    CmdCommand::Bits cmd = tcu.regs().getCommand();
    if (cmd.opcode == CmdCommand::IDLE)
    {
        if (pkt)
            tcu.schedCpuResponse(pkt, tcu.clockEdge(Cycles(1)));
        return;
    }

    assert(cmdPkt == nullptr);
    cmdPkt = pkt;
    commands[static_cast<size_t>(cmd.opcode)]++;

    assert(cmd.epid < tcu.numEndpoints);
    DPRINTF(TcuCmd, "Starting command %s with EP=%u, arg0=%#lx\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid, cmd.arg0);

    switch (cmd.opcode)
    {
        case CmdCommand::SEND:
        case CmdCommand::REPLY:
            tcu.msgUnit->startTransmission(cmd);
            break;
        case CmdCommand::READ:
            tcu.memUnit->startRead(cmd);
            break;
        case CmdCommand::WRITE:
            tcu.memUnit->startWrite(cmd);
            break;
        case CmdCommand::FETCH_MSG:
        {
            Addr offset;
            TcuError res = tcu.msgUnit->fetchMessage(cmd.epid, &offset);
            if (res == TcuError::NONE)
                tcu.regs().set(UnprivReg::ARG1, offset);
            finishCommand(res);
        }
        break;
        case CmdCommand::ACK_MSG:
        {
            TcuError res = tcu.msgUnit->ackMessage(cmd.epid, cmd.arg0);
            finishCommand(res);
        }
        break;
        case CmdCommand::SLEEP:
        {
            int ep = cmd.arg0 & 0xFFFF;
            if (!tcu.startSleep(ep))
                finishCommand(TcuError::NONE);
        }
        break;
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (cmdPkt && cmd.opcode != CmdCommand::SLEEP)
    {
        if (tcu.connector->canSuspendCmds())
            tcu.startSleep(Tcu::INVALID_EP_ID);
        else
        {
            tcu.schedCpuResponse(cmdPkt, tcu.clockEdge(Cycles(1)));
            cmdPkt = nullptr;
        }
    }
}

void
TcuCommands::abortCommand()
{
    CmdCommand::Bits cmd = tcu.regs().getCommand();

    // if we've already scheduled finishCommand, consider the command done
    if (!cmdFinish && cmd.opcode != CmdCommand::IDLE)
    {
        // if we are waiting for a NoC response, just wait until we receive it.
        // we deem this acceptable, because these remotely running transfers
        // never cause page faults and thus complete in short amounts of time.
        if (cmdIsRemote)
        {
            // SEND/REPLY needs to finish successfully as soon as we've sent
            // out the message.
            if (cmd.opcode != CmdCommand::SEND &&
                cmd.opcode != CmdCommand::REPLY)
                abort = AbortType::REMOTE;
        }
        // otherwise, abort it locally. this is done for all commands, because
        // all can cause page faults locally.
        else
        {
            abort = AbortType::LOCAL;

            auto res = tcu.xferUnit->tryAbortCommand();
            // if the current command used the xferUnit, we're done
            if (res == XferUnit::AbortResult::ABORTED)
                scheduleFinishOp(Cycles(1), TcuError::ABORT);
        }

        if (abort != AbortType::NONE)
        {
            DPRINTF(TcuCmd, "Aborting command %s with EP=%u\n",
                    cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid);
        }
    }
    else
    {
        abort = AbortType::NONE;
        // don't do that if we'll do that in finishCommand event anyway
        if (!cmdFinish)
            finishAbort();
    }
}

void
TcuCommands::scheduleFinishOp(Cycles delay, TcuError error)
{
    if (tcu.regs().getCommand().opcode != CmdCommand::IDLE)
    {
        if (cmdFinish)
        {
            tcu.deschedule(cmdFinish);
            delete cmdFinish;
        }

        if (delay == 0)
            finishCommand(error);
        else
        {
            cmdFinish = new FinishCommandEvent(*this, error);
            tcu.schedule(cmdFinish, tcu.clockEdge(delay));
        }
    }
}

void
TcuCommands::finishCommand(TcuError error)
{
    CmdCommand::Bits cmd = tcu.regs().getCommand();

    cmdFinish = NULL;

    if (error == TcuError::NONE &&
        (cmd.opcode == CmdCommand::READ || cmd.opcode == CmdCommand::WRITE))
    {
        const CmdData::Bits data = tcu.regs().getData();
        if (data.size > 0)
        {
            if (cmd.opcode == CmdCommand::READ)
                tcu.memUnit->startRead(cmd);
            else
                tcu.memUnit->startWrite(cmd);
            return;
        }
    }

    if (cmdPkt || cmd.opcode == CmdCommand::SLEEP)
        tcu.stopSleep();

    DPRINTF(TcuCmd, "Finished command %s with EP=%u -> %u\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid,
            static_cast<uint>(error));

    // let the SW know that the command is finished
    cmd = 0;
    cmd.error = static_cast<unsigned>(error);
    cmd.opcode = CmdCommand::IDLE;
    tcu.regs().set(UnprivReg::COMMAND, cmd);

    if (cmdPkt)
        tcu.schedCpuResponse(cmdPkt, tcu.clockEdge(Cycles(1)));
    cmdPkt = NULL;

    // finish command abortion, if there is any
    finishAbort();
}

void
TcuCommands::executePrivCommand(PacketPtr pkt)
{
    PrivCommand::Bits cmd = tcu.regs().get(PrivReg::PRIV_CMD);

    privCommands[static_cast<size_t>(cmd.opcode)]++;

    DPRINTF(TcuCmd, "Executing privileged command %s with arg0=%p\n",
            privCmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg0);

    Cycles delay(1);

    switch (cmd.opcode)
    {
        case PrivCommand::IDLE:
            break;
        case PrivCommand::INV_PAGE:
            if (tcu.tlb())
            {
                uint16_t asid = cmd.arg0 >> 32;
                Addr virt = cmd.arg0 & 0xFFFFFFFF;
                tcu.tlb()->remove(virt, asid);
            }
            break;
        case PrivCommand::INV_TLB:
            if (tcu.tlb())
                tcu.tlb()->clear();
            break;
        case PrivCommand::INS_TLB:
            if (tcu.tlb())
            {
                uint16_t asid = cmd.arg0 >> 32;
                Addr virt = cmd.arg0 & 0xFFFFF000;
                uint flags = cmd.arg0 & 0x1F;
                Addr phys = tcu.regs().get(PrivReg::PRIV_CMD_ARG1);
                tcu.tlb()->insert(virt, asid, NocAddr(phys), flags);
            }
            break;
        case PrivCommand::XCHG_VPE:
        {
            RegFile::reg_t old = tcu.regs().get(PrivReg::CUR_VPE);
            tcu.regs().set(PrivReg::OLD_VPE, old);
            tcu.regs().set(PrivReg::CUR_VPE, cmd.arg0 & 0xFFFFFFFF);
            break;
        }
        case PrivCommand::FLUSH_CACHE:
            delay += tcu.flushInvalCaches(true);
            break;
        case PrivCommand::SET_TIMER:
            tcu.restartTimer(cmd.arg0);
            break;
        case PrivCommand::ABORT_CMD:
            privCmdPkt = pkt;
            pkt = nullptr;
            abortCommand();
            return;
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (pkt)
        tcu.schedCpuResponse(pkt, tcu.clockEdge(delay));

    DPRINTF(TcuCmd, "Finished privileged command %s with res=0\n",
            privCmdNames[static_cast<size_t>(cmd.opcode)]);

    // set privileged command back to IDLE
    cmd.arg0 = 0;
    cmd.opcode = PrivCommand::IDLE;
    tcu.regs().set(PrivReg::PRIV_CMD, cmd);
}

void
TcuCommands::finishAbort()
{
    if (privCmdPkt != nullptr)
    {
        tcu.schedCpuResponse(privCmdPkt, tcu.clockEdge(Cycles(1)));
        privCmdPkt = nullptr;

        PrivCommand::Bits cmd = tcu.regs().get(PrivReg::PRIV_CMD);
        cmd.arg0 = static_cast<RegFile::reg_t>(abort);

        DPRINTF(TcuCmd, "Finished privileged command %s with res=%d\n",
                privCmdNames[static_cast<size_t>(cmd.opcode)],
                cmd.arg0);

        cmd.opcode = PrivCommand::IDLE;
        tcu.regs().set(PrivReg::PRIV_CMD, cmd);

        abort = AbortType::NONE;
    }
}

void
TcuCommands::executeExtCommand(PacketPtr pkt)
{
    ExtCommand::Bits cmd = tcu.regs().get(ExtReg::EXT_CMD);

    extCommands[static_cast<size_t>(cmd.opcode)]++;

    Cycles delay(1);

    DPRINTF(TcuCmd, "Executing external command %s with arg=%p\n",
            extCmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg);

    switch (cmd.opcode)
    {
        case ExtCommand::IDLE:
            break;
        case ExtCommand::INV_EP:
        {
            epid_t epid = cmd.arg & 0xFFFF;
            bool force = !!(cmd.arg & (1 << 16));
            unsigned unreadMask;
            TcuError res = tcu.regs().invalidate(epid, force, &unreadMask);
            cmd.error = static_cast<uint>(res);
            cmd.arg = unreadMask;
            break;
        }
        case ExtCommand::RESET:
            delay += tcu.reset(cmd.arg != 0);
            break;
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (pkt)
        tcu.schedNocResponse(pkt, tcu.clockEdge(delay));

    if (cmd.error != static_cast<uint>(TcuError::NONE))
    {
        DPRINTF(TcuCmd, "External command %s failed (%u)\n",
                extCmdNames[static_cast<size_t>(cmd.opcode)],
                static_cast<uint>(cmd.error));
    }

    // set external command back to IDLE
    cmd.opcode = ExtCommand::IDLE;
    tcu.regs().set(ExtReg::EXT_CMD, cmd);
}
