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

#include "cpu/dtu-accel/ctxswsm.hh"

AccelCtxSwSM::AccelCtxSwSM(DtuAccel *_accel)
    : accel(_accel),
      state(SET_VPE), stateChanged(), switched(), vpe_id(IDLE_VPE)
{
}

std::string
AccelCtxSwSM::stateName() const
{
    const char *names[] =
    {
        "SET_VPE",
        "FETCH_MSG",
        "READ_MSG_ADDR",
        "READ_MSG",
        "STORE_REPLY",
        "SEND_REPLY",
        "REPLY_WAIT",
        "DONE",
    };
    return names[static_cast<size_t>(state)];
}

PacketPtr
AccelCtxSwSM::tick()
{
    PacketPtr pkt = nullptr;

    switch(state)
    {
        case State::SET_VPE:
        {
            pkt = accel->createDtuRegPkt(
                accel->getRegAddr(PrivReg::CUR_VPE), sizeof(uint64_t), MemCmd::WriteReq
            );
            *pkt->getPtr<uint64_t>() = OUR_VPE;
            break;
        }

        case State::FETCH_MSG:
        {
            Addr regAddr = accel->getRegAddr(CmdReg::COMMAND);
            uint64_t value = Dtu::Command::FETCH_MSG | (EP_RECV << 4);
            pkt = accel->createDtuRegPkt(regAddr, value, MemCmd::WriteReq);
            break;
        }
        case State::READ_MSG_ADDR:
        {
            Addr regAddr = accel->getRegAddr(CmdReg::OFFSET);
            pkt = accel->createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::READ_MSG:
        {
            pkt = accel->createPacket(msgAddr, MSG_SIZE, MemCmd::ReadReq);
            break;
        }

        case State::STORE_REPLY:
        {
            reply.res = 0;
            pkt = accel->createPacket(msgAddr, sizeof(reply), MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(), (char*)&reply, sizeof(reply));
            break;
        }
        case State::SEND_REPLY:
        {
            pkt = accel->createDtuCmdPkt(Dtu::Command::REPLY,
                                         EP_RECV,
                                         msgAddr,
                                         sizeof(reply),
                                         msgAddr);
            break;
        }
        case State::REPLY_WAIT:
        {
            Addr regAddr = accel->getRegAddr(CmdReg::COMMAND);
            pkt = accel->createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::DONE:
        {
            pkt = accel->createDtuRegPkt(
                accel->getRegAddr(PrivReg::CUR_VPE), sizeof(uint64_t), MemCmd::WriteReq
            );
            *pkt->getPtr<uint64_t>() = vpe_id;
            break;
        }
    }

    return pkt;
}

bool
AccelCtxSwSM::handleMemResp(PacketPtr pkt)
{
    auto lastState = state;

    switch(state)
    {
        case State::SET_VPE:
        {
            state = State::FETCH_MSG;
            break;
        }

        case State::FETCH_MSG:
        {
            state = State::READ_MSG_ADDR;
            break;
        }
        case State::READ_MSG_ADDR:
        {
            const RegFile::reg_t *regs = pkt->getConstPtr<RegFile::reg_t>();
            if(regs[0])
            {
                msgAddr = regs[0];
                state = State::READ_MSG;
            }
            else
                state = State::DONE;
            break;
        }
        case State::READ_MSG:
        {
            const uint64_t *args =
                reinterpret_cast<const uint64_t*>(pkt->getConstPtr<uint8_t>() + sizeof(MessageHeader));

            vpe_id = args[2];
            if (args[3] == Operation::START)
                switched = true;
            else if (args[3] == Operation::STOP)
                vpe_id = IDLE_VPE;
            state = State::STORE_REPLY;
            break;
        }

        case State::STORE_REPLY:
        {
            state = State::SEND_REPLY;
            break;
        }
        case State::SEND_REPLY:
        {
            state = State::REPLY_WAIT;
            break;
        }
        case State::REPLY_WAIT:
        {
            Dtu::Command::Bits cmd =
                *reinterpret_cast<const RegFile::reg_t*>(pkt->getConstPtr<uint8_t>());
            if (cmd.opcode == 0)
                state = State::DONE;
            break;
        }

        case State::DONE:
        {
            if (switched)
            {
                accel->setSwitched();
                switched = false;
            }
            state = State::SET_VPE;
            return true;
        }
    }

    stateChanged = state != lastState;

    return false;
}
