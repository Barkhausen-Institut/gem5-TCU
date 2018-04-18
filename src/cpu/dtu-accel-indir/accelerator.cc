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

#include "cpu/dtu-accel-indir/accelerator.hh"
#include "debug/DtuAccelInDir.hh"
#include "debug/DtuAccelInDirState.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/regfile.hh"
#include "sim/dtu_memory.hh"

#include <iomanip>

static const char *stateNames[] =
{
    "IDLE",

    "FETCH_MSG",
    "READ_MSG_ADDR",
    "READ_MSG",

    "WRITE_DATA",
    "WRITE_DATA_WAIT",

    "STORE_REPLY",
    "SEND_REPLY",
    "REPLY_WAIT",
    "REPLY_ERROR",

    "CTXSW",

    "SYSCALL",
};

DtuAccelInDir::DtuAccelInDir(const DtuAccelInDirParams *p)
  : DtuAccel(p),
    bufSize(p->buf_size),
    irqPending(false),
    memPending(false),
    state(State::IDLE),
    lastState(State::CTXSW),    // something different
    msgAddr(),
    ctx(),
    sysc(this),
    yield(this, &sysc),
    ctxsw(this)
{
    static_assert((sizeof(stateNames) / sizeof(stateNames[0]) ==
                  static_cast<size_t>(State::SYSCALL) + 1), "Missmatch");
    yield.start();
}

std::string DtuAccelInDir::getStateName() const
{
    std::ostringstream os;
    os << stateNames[static_cast<size_t>(state)];
    if (state == State::IDLE)
        os << ":" << yield.stateName();
    else if (state == State::SYSCALL)
        os << ":" << sysc.stateName();
    else if (state == State::CTXSW)
        os << ":" << ctxsw.stateName();
    return os.str();
}

void
DtuAccelInDir::completeRequest(PacketPtr pkt)
{
    Request* req = pkt->req;

    if (state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelInDirState, "[%s] Got response from memory\n",
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
                state = State::READ_MSG_ADDR;
                break;
            }
            case State::READ_MSG_ADDR:
            {
                const RegFile::reg_t *regs = pkt->getConstPtr<RegFile::reg_t>();
                if(regs[0])
                {
                    msgAddr = regs[0];
                    DPRINTF(DtuAccelInDir, "Received message @ %p\n", msgAddr);
                    state = State::READ_MSG;
                }
                else
                {
                    yield.start();
                    state = State::IDLE;
                }
                break;
            }
            case State::READ_MSG:
            {
                const uint64_t *args =
                    reinterpret_cast<const uint64_t*>(
                        pkt_data + sizeof(MessageHeader));

                DPRINTF(DtuAccelInDir, "  op=%d arg0=%#llx arg1=%#llx\n",
                    args[0], args[1], args[2]);

                dataSize = args[1];
                reply.msg.count = dataSize;

                if (static_cast<Operation>(args[0]) == Operation::COMPUTE)
                {
                    size_t blocks = (dataSize + BLOCK_SIZE - 1) / BLOCK_SIZE;
                    size_t compTime = args[2];
                    delay = Cycles(blocks * compTime);
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
                Dtu::Command::Bits cmd =
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
                Dtu::Command::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    if (cmd.error == 0)
                        state = State::CTXSW;
                    else
                        state = State::REPLY_ERROR;
                }
                break;
            }
            case State::REPLY_ERROR:
            {
                sysc.start(sizeof(reply));
                syscNext = State::CTXSW;
                state = State::SYSCALL;
                break;
            }

            case State::SYSCALL:
            {
                if(sysc.handleMemResp(pkt))
                    state = syscNext;
                break;
            }
        }
    }

    memPending = false;
    freePacket(pkt);

    // kick things into action again
    schedule(tickEvent, clockEdge(delay));
}

void
DtuAccelInDir::interrupt()
{
    irqPending = true;

    if (ctxsw.isWaiting())
    {
        ctxsw.restart();
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void
DtuAccelInDir::reset()
{
    irqPending = false;

    yield.start(false);
    state = State::IDLE;

    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuAccelInDir::tick()
{
    PacketPtr pkt = nullptr;

    if (state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelInDirState, "[%s] tick\n",
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
                Addr regAddr = getRegAddr(CmdReg::COMMAND);
                uint64_t value = Dtu::Command::FETCH_MSG | (EP_RECV << 4);
                pkt = createDtuRegPkt(regAddr, value, MemCmd::WriteReq);
            }
            break;
        }
        case State::READ_MSG_ADDR:
        {
            Addr regAddr = getRegAddr(CmdReg::OFFSET);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::READ_MSG:
        {
            pkt = createPacket(msgAddr, MSG_SIZE, MemCmd::ReadReq);
            break;
        }

        case State::WRITE_DATA:
        {
            pkt = createDtuCmdPkt(Dtu::Command::WRITE,
                                  EP_OUT,
                                  BUF_ADDR,
                                  dataSize,
                                  0);
            break;
        }
        case State::WRITE_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::STORE_REPLY:
        {
            pkt = createPacket(BUF_ADDR,
                               sizeof(reply.msg),
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(), (char*)&reply.msg, sizeof(reply.msg));
            break;
        }
        case State::SEND_REPLY:
        {
            pkt = createDtuCmdPkt(Dtu::Command::REPLY,
                                  EP_RECV,
                                  BUF_ADDR,
                                  sizeof(reply.msg),
                                  msgAddr);
            break;
        }
        case State::REPLY_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::REPLY_ERROR:
        {
            reply.sys.opcode = SyscallSM::Operation::FORWARD_REPLY;
            reply.sys.cap = CAP_RBUF;
            reply.sys.msgaddr = msgAddr;
            reply.sys.event = 0;
            reply.sys.len = sizeof(reply.msg);
            reply.sys.event = 0;

            pkt = createPacket(MSG_ADDR, sizeof(reply), MemCmd::WriteReq);
            memcpy(pkt->getPtr<void>(), &reply, sizeof(reply));
            break;
        }

        case State::SYSCALL:
        {
            pkt = sysc.tick();
            break;
        }
    }

    if (pkt != nullptr)
    {
        memPending = true;
        sendPkt(pkt);
    }
}

DtuAccelInDir*
DtuAccelInDirParams::create()
{
    return new DtuAccelInDir(this);
}
