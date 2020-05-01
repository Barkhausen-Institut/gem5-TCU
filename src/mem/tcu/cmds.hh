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

#ifndef __MEM_TCU_CMDS_HH__
#define __MEM_TCU_CMDS_HH__

#include "mem/tcu/base.hh"
#include "mem/tcu/error.hh"

class Tcu;

class TcuCommands
{
  public:

    enum class AbortType
    {
        NONE,
        LOCAL,
        REMOTE,
    };

  public:

    TcuCommands(Tcu &_tcu);

    const std::string name() const;

    void regStats();

    void startCommand(RegFile::Result written, PacketPtr pkt, Tick when);

    void stopCommand();

    void scheduleFinishOp(Cycles delay, TcuError error = TcuError::NONE);

    void setRemoteCommand(bool remote)
    {
        cmdIsRemote = remote;
    }

    bool isCommandAborting() const
    {
        return abort != AbortType::NONE;
    }

  private:

    void executeCommand(PacketPtr pkt);

    void abortCommand();

    void executePrivCommand(PacketPtr pkt);

    void finishAbort();

    void executeExtCommand(PacketPtr pkt);

    void finishCommand(TcuError error);

  private:

    struct CmdEvent : public Event
    {
        TcuCommands& cmds;

        CmdEvent(TcuCommands& _cmds)
            : cmds(_cmds)
        {}

        const std::string name() const override;
    };

    struct ExecCmdEvent : public CmdEvent
    {
        PacketPtr pkt;

        ExecCmdEvent(TcuCommands& _cmds, PacketPtr _pkt)
            : CmdEvent(_cmds), pkt(_pkt)
        {}

        void process() override
        {
            cmds.executeCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecCmdEvent"; }
    };

    struct ExecPrivCmdEvent : public CmdEvent
    {
        PacketPtr pkt;

        ExecPrivCmdEvent(TcuCommands& _cmds, PacketPtr _pkt)
            : CmdEvent(_cmds), pkt(_pkt)
        {}

        void process() override
        {
            cmds.executePrivCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecPrivCmdEvent"; }
    };

    struct ExecExtCmdEvent : public CmdEvent
    {
        PacketPtr pkt;

        ExecExtCmdEvent(TcuCommands& _cmds, PacketPtr _pkt)
            : CmdEvent(_cmds), pkt(_pkt)
        {}

        void process() override
        {
            cmds.executeExtCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecExtCmdEvent"; }
    };

    struct FinishCommandEvent : public CmdEvent
    {
        TcuError error;

        FinishCommandEvent(TcuCommands& _cmds, TcuError _error = TcuError::NONE)
            : CmdEvent(_cmds), error(_error)
        {}

        void process() override
        {
            cmds.finishCommand(error);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "FinishCommandEvent"; }
    };

    Tcu &tcu;

    PacketPtr cmdPkt;
    PacketPtr privCmdPkt;
    FinishCommandEvent *cmdFinish;
    AbortType abort;
    bool cmdIsRemote;

  public:

    Stats::Vector commands;
    Stats::Vector privCommands;
    Stats::Vector extCommands;

};

#endif // __MEM_TCU_CMDS_HH__
