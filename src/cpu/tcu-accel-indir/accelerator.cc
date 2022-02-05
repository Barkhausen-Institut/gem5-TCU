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

#include "cpu/tcu-accel-indir/accelerator.hh"
#include "debug/TcuAccelInDir.hh"
#include "debug/TcuAccelInDirState.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/reg_file.hh"

#include <iomanip>

static const char *stateNames[] =
{
    "IDLE",

    "FETCH_MSG",
    "FETCH_MSG_WAIT",
    "READ_MSG_ADDR",
    "READ_MSG",

    "WRITE_DATA",
    "WRITE_DATA_WAIT",

    "STORE_REPLY",
    "SEND_REPLY",
    "REPLY_WAIT",

    "CTXSW",
};

TcuAccelInDir::TcuAccelInDir(const TcuAccelInDirParams &p)
  : TcuAccel(p),
    bufSize(p.buf_size),
    irqPending(false),
    memPending(false),
    state(State::IDLE),
    lastState(State::CTXSW),    // something different
    msgAddr(),
    yield(this),
    ctxsw(this)
{
    static_assert((sizeof(stateNames) / sizeof(stateNames[0]) ==
                  static_cast<size_t>(State::CTXSW) + 1), "Missmatch");
}

std::string TcuAccelInDir::getStateName() const
{
    std::ostringstream os;
    os << stateNames[static_cast<size_t>(state)];
    if (state == State::CTXSW)
        os << ":" << ctxsw.stateName();
    return os.str();
}

void
TcuAccelInDir::completeRequest(PacketPtr pkt)
{
    RequestPtr req = pkt->req;

    if (state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()))
    {
        DPRINTF(TcuAccelInDirState, "[%s] Got response from memory\n",
            getStateName().c_str());
        lastState = state;
    }

    const uint8_t *pkt_data = pkt->getConstPtr<uint8_t>();

    Cycles delay(1);
    if (pkt->isError())
    {
        warn("%s access failed at %#x\n",
             pkt->isWrite() ? "Write" : "Read", req->getPaddr());
    }
    else
    {
        switch(state)
        {
            case State::IDLE:
            {
                if (yield.handleMemResp(pkt))
                {
                    if (irqPending)
                        irqPending = false;
                    state = State::CTXSW;
                }
                break;
            }

            case State::CTXSW:
            {
                if(ctxsw.handleMemResp(pkt))
                    state = State::FETCH_MSG;
                break;
            }

            case State::FETCH_MSG:
            {
                state = State::FETCH_MSG_WAIT;
                break;
            }
            case State::FETCH_MSG_WAIT:
            {
                CmdCommand::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                    state = State::READ_MSG_ADDR;
                break;
            }
            case State::READ_MSG_ADDR:
            {
                const RegFile::reg_t *regs = pkt->getConstPtr<RegFile::reg_t>();
                if(regs[0] != static_cast<RegFile::reg_t>(-1))
                {
                    msgAddr = rbufAddr() + regs[0];
                    DPRINTF(TcuAccelInDir, "Received message @ %p\n", msgAddr);
                    state = State::READ_MSG;
                }
                else
                {
                    state = State::IDLE;
                }
                break;
            }
            case State::READ_MSG:
            {
                const uint64_t *args =
                    reinterpret_cast<const uint64_t*>(
                        pkt_data + sizeof(MessageHeader));

                DPRINTF(TcuAccelInDir, "  op=%d arg0=%#llx arg1=%#llx\n",
                    args[0], args[1], args[2]);

                dataSize = args[1];
                reply.count = dataSize;

                if (static_cast<Operation>(args[0]) == Operation::COMPUTE)
                {
                    size_t compTime = args[2];
                    delay = Cycles((compTime * dataSize) / BLOCK_SIZE);
                    state = State::STORE_REPLY;
                }
                else
                    state = State::WRITE_DATA;
                break;
            }

            case State::WRITE_DATA:
            {
                state = State::WRITE_DATA_WAIT;
                break;
            }
            case State::WRITE_DATA_WAIT:
            {
                CmdCommand::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
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
                CmdCommand::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                    state = State::CTXSW;
                break;
            }
        }
    }

    memPending = false;
    tcuif().freePacket(pkt);

    // kick things into action again
    schedule(tickEvent, clockEdge(delay));
}

void
TcuAccelInDir::wakeup()
{
    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
TcuAccelInDir::interrupt()
{
    irqPending = true;
}

void
TcuAccelInDir::reset()
{
    irqPending = false;

    state = State::IDLE;
}

void
TcuAccelInDir::tick()
{
    PacketPtr pkt = nullptr;

    if (state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(TcuAccelInDirState, "[%s] tick\n",
            getStateName().c_str());
        lastState = state;
    }

    switch(state)
    {
        case State::IDLE:
        {
            pkt = yield.tick();
            break;
        }

        case State::CTXSW:
        {
            pkt = ctxsw.tick();
            break;
        }

        case State::FETCH_MSG:
        {
            if (irqPending)
            {
                irqPending = false;
                state = State::CTXSW;
                schedule(tickEvent, clockEdge(Cycles(1)));
            }
            else
            {
                pkt = tcuif().createTcuCmdPkt(
                    CmdCommand::create(CmdCommand::FETCH_MSG, EP_RECV),
                    0
                );
            }
            break;
        }
        case State::READ_MSG_ADDR:
        {
            Addr regAddr = TcuIf::getRegAddr(UnprivReg::ARG1);
            pkt = tcuif().createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::READ_MSG:
        {
            pkt = tcuif().createPacket(msgAddr, MSG_SIZE, MemCmd::ReadReq);
            break;
        }

        case State::WRITE_DATA:
        {
            pkt = tcuif().createTcuCmdPkt(
                CmdCommand::create(CmdCommand::WRITE, EP_OUT),
                CmdData::create(bufferAddr(), dataSize)
            );
            break;
        }

        case State::STORE_REPLY:
        {
            pkt = tcuif().createPacket(bufferAddr(),
                                       sizeof(reply),
                                       MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(), (char*)&reply, sizeof(reply));
            break;
        }
        case State::SEND_REPLY:
        {
            pkt = tcuif().createTcuCmdPkt(
                CmdCommand::create(CmdCommand::REPLY, EP_RECV,
                                   msgAddr - rbufAddr()),
                CmdData::create(bufferAddr(), sizeof(reply))
            );
            break;
        }

        case State::FETCH_MSG_WAIT:
        case State::WRITE_DATA_WAIT:
        case State::REPLY_WAIT:
        {
            Addr regAddr = TcuIf::getRegAddr(UnprivReg::COMMAND);
            pkt = tcuif().createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
    }

    if (pkt != nullptr)
    {
        memPending = true;
        sendPkt(pkt);
    }
}
