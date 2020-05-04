/*
 * Copyright (c) 2016, Nils Asmussen
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

#include "cpu/tcu-accel/syscallsm.hh"

std::string
SyscallSM::stateName() const
{
    const char *names[] =
    {
        "SEND", "WAIT", "FETCH", "READ_ADDR", "ACK"
    };
    return names[static_cast<size_t>(state)];
}

PacketPtr
SyscallSM::tick()
{
    PacketPtr pkt = nullptr;

    switch(state)
    {
        case State::SYSC_SEND:
        {
            pkt = accel->createTcuCmdPkt(
                CmdCommand::create(CmdCommand::SEND, TcuAccel::EP_SYSS,
                                   TcuAccel::EP_SYSR),
                CmdData::create(accel->sendMsgAddr(), syscallSize)
            );
            break;
        }
        case State::SYSC_WAIT:
        {
            Addr regAddr = accel->getRegAddr(UnprivReg::COMMAND);
            pkt = accel->createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::SYSC_FETCH:
        {
            if (fetched)
                return nullptr;

            pkt = accel->createTcuCmdPkt(
                CmdCommand::create(CmdCommand::FETCH_MSG, TcuAccel::EP_SYSR),
                0
            );
            break;
        }
        case State::SYSC_READ_ADDR:
        {
            Addr regAddr = accel->getRegAddr(UnprivReg::ARG1);
            pkt = accel->createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::SYSC_ACK:
        {
            pkt = accel->createTcuCmdPkt(
                CmdCommand::create(CmdCommand::ACK_MSG, TcuAccel::EP_SYSR,
                                   replyAddr - RBUF_ADDR),
                0
            );
            break;
        }
    }

    return pkt;
}

bool
SyscallSM::handleMemResp(PacketPtr pkt)
{
    auto lastState = state;

    switch(state)
    {
        case State::SYSC_SEND:
        {
            state = State::SYSC_WAIT;
            break;
        }
        case State::SYSC_WAIT:
        {
            auto data = pkt->getConstPtr<RegFile::reg_t>();
            CmdCommand::Bits cmd =
                *reinterpret_cast<const RegFile::reg_t*>(data);
            if (cmd.opcode == 0)
            {
                auto err = static_cast<TcuError>((long)cmd.error);
                if (!waitForReply || err != TcuError::NONE)
                    return true;
                state = State::SYSC_FETCH;
            }
            break;
        }
        case State::SYSC_FETCH:
        {
            state = State::SYSC_READ_ADDR;
            break;
        }
        case State::SYSC_READ_ADDR:
        {
            const RegFile::reg_t *regs = pkt->getConstPtr<RegFile::reg_t>();
            if(regs[0] != static_cast<RegFile::reg_t>(-1))
            {
                replyAddr = regs[0] + RBUF_ADDR;
                state = State::SYSC_ACK;
            }
            else
            {
                fetched = true;
                state = State::SYSC_FETCH;
            }
            break;
        }
        case State::SYSC_ACK:
        {
            return true;
        }
    }

    stateChanged = state != lastState;

    return false;
}
