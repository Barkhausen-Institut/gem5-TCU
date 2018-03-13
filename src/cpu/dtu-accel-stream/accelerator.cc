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

#include "cpu/dtu-accel-stream/accelerator.hh"
#include "debug/DtuAccelStream.hh"
#include "debug/DtuAccelStreamState.hh"
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

    "INOUT_START",
    "INOUT_SEND",
    "INOUT_SEND_WAIT",
    "INOUT_SEND_ERROR",
    "INOUT_ACK",

    "READ_DATA",
    "READ_DATA_WAIT",
    "COMPUTE",

    "WRITE_DATA",
    "WRITE_DATA_WAIT",


    "CTXSW",

    "SYSCALL",

    "EXIT",
};

DtuAccelStream::DtuAccelStream(const DtuAccelStreamParams *p)
  : DtuAccel(p),
    irqPending(false),
    memPending(false),
    state(State::IDLE),
    lastState(State::FETCH_MSG), // something different
    ctx(),
    bufSize(p->buf_size),
    sysc(this),
    yield(this, &sysc),
    logic(this, p->algorithm),
    ctxsw(this),
    ctxSwPerformed()
{
    ctx.flags = Flags::INPUT;
    yield.start();
}

std::string DtuAccelStream::getStateName() const
{
    std::ostringstream os;
    os << stateNames[static_cast<size_t>(state)];
    if (state == State::IDLE)
        os << ":" << yield.stateName();
    else if (state == State::SYSCALL)
        os << ":" << sysc.stateName();
    else if (state == State::COMPUTE)
        os << ":" << logic.stateName();
    else if (state == State::CTXSW)
        os << ":" << ctxsw.stateName();
    return os.str();
}

void
DtuAccelStream::completeRequest(PacketPtr pkt)
{
    Request* req = pkt->req;

    if (state != lastState ||
        (state == State::COMPUTE && logic.hasStateChanged()) ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelStreamState, "[%s] Got response from memory\n",
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
                {
                    // if we didn't perform a context switch, go back to our
                    // previous state
                    if ((ctx.flags & Flags::INTRPT) && !ctxSwPerformed)
                    {
                        state = State::READ_DATA;
                        ctx.flags &= ~Flags::INTRPT;
                    }
                    else
                    {
                        state = (ctx.flags & Flags::WAIT) ? State::FETCH_MSG
                                                          : State::INOUT_START;
                    }
                }
                break;
            }

            case State::INOUT_START:
            {
                state = State::INOUT_SEND;
                break;
            }
            case State::INOUT_SEND:
            {
                state = State::INOUT_SEND_WAIT;
                break;
            }
            case State::INOUT_SEND_WAIT:
            {
                Dtu::Command::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    auto err = static_cast<Dtu::Error>((long)cmd.error);
                    if (err == Dtu::Error::VPE_GONE)
                        state = State::INOUT_SEND_ERROR;
                    else
                    {
                        if (err == Dtu::Error::NONE)
                            ctx.flags |= Flags::WAIT;
                        // ignore other errors
                        yield.start();
                        state = State::IDLE;
                    }
                }
                break;
            }
            case State::INOUT_SEND_ERROR:
            {
                sysc.start(sizeof(rdwr_msg));
                syscNext = State::IDLE;
                state = State::SYSCALL;
                break;
            }
            case State::INOUT_ACK:
            {
                if (ctx.flags & Flags::EXIT)
                    state = State::EXIT;
                else
                {
                    state = (ctx.flags & Flags::INPUT) ? State::READ_DATA
                                                       : State::WRITE_DATA;
                }
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
                    ctx.msgAddr = regs[0];
                    DPRINTF(DtuAccelStream,
                            "Received message @ %p\n", ctx.msgAddr);
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
                auto head = reinterpret_cast<const MessageHeader*>(pkt_data);
                auto args = reinterpret_cast<const uint64_t*>(
                    pkt_data + sizeof(MessageHeader));

                switch(head->label)
                {
                    case LBL_IN_REPLY:
                    {
                        DPRINTF(
                            DtuAccelStream,
                            "  input reply(err=%#llx, off=%#llx, len=%#llx)\n",
                            args[0], args[1], args[2]
                        );

                        ctx.inOff = args[1];
                        ctx.inLen = args[2];
                        ctx.inPos = 0;
                        ctx.flags &= ~Flags::WAIT;
                        if (ctx.inLen == 0)
                        {
                            ctx.flags |= Flags::EXIT;
                            ctx.flags &= ~Flags::INPUT;
                            state = State::INOUT_START;
                        }
                        else
                            state = State::INOUT_ACK;
                        break;
                    }

                    case LBL_OUT_REPLY:
                    {
                        DPRINTF(
                            DtuAccelStream,
                            "  output reply(err=%#llx, off=%#llx, len=%#llx)\n",
                            args[0], args[1], args[2]
                        );

                        ctx.outOff = args[1];
                        ctx.outLen = args[2];
                        ctx.outPos = 0;
                        ctx.flags &= ~Flags::WAIT;
                        state = State::INOUT_ACK;
                        break;
                    }

                    default:
                        panic("Unexpected label: %#llx\n", head->label);
                }
                break;
            }

            case State::READ_DATA:
            {
                state = State::READ_DATA_WAIT;
                break;
            }
            case State::READ_DATA_WAIT:
            {
                Dtu::Command::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    logic.start(ctx.lastSize, Cycles(4096) /* TODO */);
                    state = State::COMPUTE;
                }
                break;
            }
            case State::COMPUTE:
            {
                if(logic.handleMemResp(pkt, &delay))
                {
                    ctx.lastSize = logic.outDataSize();
                    ctx.flags &= ~Flags::INPUT;
                    state = State::INOUT_START;
                }
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
                {
                    ctx.flags |= Flags::INPUT;
                    state = State::INOUT_START;
                }
                break;
            }


            case State::SYSCALL:
            {
                if(sysc.handleMemResp(pkt))
                    state = syscNext;
                break;
            }

            case State::EXIT:
            {
                sysc.start(sizeof(exit_msg), false);
                syscNext = State::IDLE;
                state = State::SYSCALL;
                // in case we don't restore any state, reset flags
                ctx.flags = Flags::INPUT;
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
DtuAccelStream::wakeup()
{
    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuAccelStream::interrupt()
{
    // if we're waiting after SAVE, start with the RESTORE
    if (ctxsw.isWaiting())
    {
        ctxsw.restart();
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
    // we'll handle the IRQ later
    else
        irqPending = true;
}

void
DtuAccelStream::reset()
{
    irqPending = false;

    yield.start(false);
    state = State::IDLE;

    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuAccelStream::tick()
{
    PacketPtr pkt = nullptr;

    // after a context switch, continue at then position we left off
    if (ctxSwPerformed)
    {
        if (ctx.flags & Flags::INTRPT)
            state = State::READ_DATA;
        else if (ctx.flags & Flags::WAIT)
            state = State::FETCH_MSG;
        else
            state = State::INOUT_START;
        ctxSwPerformed = false;
        ctx.flags &= ~Flags::INTRPT;
    }

    if (state == State::INOUT_START)
    {
        if (ctx.flags & Flags::EXIT)
        {
            if (ctx.outPos == ctx.outLen)
                state = State::EXIT;
        }
        else if ((ctx.flags & Flags::INPUT) && ctx.inPos < ctx.inLen)
            state = State::READ_DATA;
        else if (!(ctx.flags & Flags::INPUT) && ctx.outPos < ctx.outLen)
            state = State::WRITE_DATA;
    }

    if (state != lastState ||
        (state == State::COMPUTE && logic.hasStateChanged()) ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelStreamState, "[%s] tick\n",
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

        case State::INOUT_START:
        {
            rdwr_msg.msg.cmd = static_cast<uint64_t>(
                (ctx.flags & Flags::INPUT) ? Command::READ : Command::WRITE
            );
            rdwr_msg.msg.submit = (ctx.flags & Flags::EXIT) ? ctx.outPos : 0;
            pkt = createPacket(BUF_ADDR + bufSize,
                               sizeof(rdwr_msg.msg),
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(),
                   (char*)&rdwr_msg.msg,
                   sizeof(rdwr_msg.msg));
            break;
        }
        case State::INOUT_SEND:
        {
            pkt = createDtuCmdPkt(
                Dtu::Command::SEND,
                (ctx.flags & Flags::INPUT) ? EP_IN_SEND : EP_OUT_SEND,
                BUF_ADDR + bufSize,
                sizeof(rdwr_msg.msg),
                EP_RECV,
                (ctx.flags & Flags::INPUT) ? LBL_IN_REPLY : LBL_OUT_REPLY
            );
            break;
        }
        case State::INOUT_SEND_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::INOUT_SEND_ERROR:
        {
            uint64_t sel = (ctx.flags & Flags::INPUT) ? CAP_IN : CAP_OUT;
            rdwr_msg.sys.opcode = 16;            /* FORWARD_MSG */
            rdwr_msg.sys.sgate_sel = sel;
            rdwr_msg.sys.rgate_sel = 0xFFFF;
            rdwr_msg.sys.len = sizeof(rdwr_msg.msg);
            rdwr_msg.sys.rlabel = 0;
            rdwr_msg.sys.event = 0;

            pkt = createPacket(MSG_ADDR, sizeof(rdwr_msg), MemCmd::WriteReq);
            memcpy(pkt->getPtr<void>(), &rdwr_msg, sizeof(rdwr_msg));
            break;
        }
        case State::INOUT_ACK:
        {
            pkt = createDtuCmdPkt(Dtu::Command::ACK_MSG,
                                  EP_RECV,
                                  0,
                                  0,
                                  ctx.msgAddr);
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
            pkt = createPacket(ctx.msgAddr, MSG_SIZE, MemCmd::ReadReq);
            break;
        }

        case State::READ_DATA:
        {
            if (irqPending)
            {
                irqPending = false;
                ctx.flags |= Flags::INTRPT;
                state = State::CTXSW;
                schedule(tickEvent, clockEdge(Cycles(1)));
            }
            else
            {
                ctx.lastSize = std::min(bufSize, ctx.inLen - ctx.inPos);
                pkt = createDtuCmdPkt(Dtu::Command::READ,
                                      EP_IN_MEM,
                                      BUF_ADDR,
                                      ctx.lastSize,
                                      ctx.inOff + ctx.inPos);
                ctx.inPos += ctx.lastSize;
            }
            break;
        }
        case State::READ_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::COMPUTE:
        {
            pkt = logic.tick();
            break;
        }

        case State::WRITE_DATA:
        {
            size_t amount = std::min(ctx.lastSize, ctx.outLen - ctx.outPos);
            pkt = createDtuCmdPkt(Dtu::Command::WRITE,
                                  EP_OUT_MEM,
                                  BUF_ADDR,
                                  amount,
                                  ctx.outOff + ctx.outPos);

            ctx.outPos += amount;
            break;
        }
        case State::WRITE_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }


        case State::SYSCALL:
        {
            pkt = sysc.tick();
            break;
        }

        case State::EXIT:
        {
            exit_msg.opcode = 10;     /* VPE_CTRL */
            exit_msg.op = 3;          /* VCTRL_STOP */
            exit_msg.vpe_sel = 0;
            exit_msg.arg = 0;

            pkt = createPacket(MSG_ADDR, sizeof(exit_msg), MemCmd::WriteReq);
            memcpy(pkt->getPtr<void>(), &exit_msg, sizeof(exit_msg));
            break;
        }
    }

    if (pkt != nullptr)
    {
        memPending = true;
        sendPkt(pkt);
    }
}

DtuAccelStream*
DtuAccelStreamParams::create()
{
    return new DtuAccelStream(this);
}
