/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2021 Nils Asmussen, Barkhausen Institut
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

#ifndef __DEV_TCU_CMD_SM_HH__
#define __DEV_TCU_CMD_SM_HH__

#include "sim/sim_object.hh"
#include "mem/packet.hh"
#include "mem/tcu/tcuif.hh"

#include <string>

class CommandExecutor {
public:
    virtual ~CommandExecutor() {}

    virtual std::string execName() = 0;
    virtual void commandFinished() = 0;
    virtual void scheduleCommand(Cycles delay) = 0;
    virtual void sendMemoryReq(PacketPtr pkt, Cycles delay) = 0;
};

class CommandSM
{
  public:
    enum State
    {
        CMD_IDLE,
        CMD_SEND,
        CMD_WAIT
    };

    explicit CommandSM(TcuIf &_tcu, CommandExecutor* _exec)
        : tcu(_tcu), exec(_exec), state(CMD_IDLE), cmd(nullptr)
    {
    }

    const std::string name() const
    {
        return exec->execName() + "::CommandSM";
    }

    std::string stateName() const;

    bool isIdle() const { return state == CMD_IDLE; }

    void executeCommand(PacketPtr cmdPkt);

    void handleMemResp(PacketPtr pkt);

    void tick();

  private:
    TcuIf tcu;
    CommandExecutor *exec;
    State state;
    PacketPtr cmd;
};

#endif // __DEV_TCU_CMD_SM_HH__
