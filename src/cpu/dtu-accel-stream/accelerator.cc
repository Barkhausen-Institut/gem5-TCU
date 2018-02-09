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
    "READ_DATA",
    "READ_DATA_WAIT",
    "COMPUTE",
    "WRITE_DATA",
    "WRITE_DATA_WAIT",

    "MSG_STORE",
    "MSG_SEND",
    "MSG_WAIT",
    "MSG_IDLE",
    "MSG_ERROR",

    "REPLY_SEND",
    "REPLY_WAIT",
    "REPLY_ERROR",

    "CTXSW",

    "SYSCALL",
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
    ctxsw(this)
{
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

    if (state != lastState)
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
                    ctx.msgAddr = regs[0];
                    DPRINTF(DtuAccelStream, "Received message @ %p\n", ctx.msgAddr);
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

                Command cmd = static_cast<Command>(args[0]);
                if (cmd == Command::INIT)
                {
                    DPRINTF(DtuAccelStream,
                            "  init(outsz=%#llx, reportsz=%#llx, comptime=%#llx)\n",
                            args[1], args[2], args[3]);

                    ctx.outSize = args[1];
                    ctx.reportSize = args[2];
                    ctx.compTime = Cycles(args[3]);
                    ctx.accSize = 0;
                    ctx.outOff = 0;
                    state = State::REPLY_SEND;
                }
                else
                {
                    DPRINTF(DtuAccelStream,
                            "  update(off=%lld len=%#llx eof=%lld)\n",
                            args[1], args[2], args[3]);

                    ctx.off = args[1];
                    ctx.inOff = 0;
                    ctx.dataSize = args[2];
                    ctx.eof = args[3];
                    if (ctx.dataSize == 0)
                    {
                        msg.msg.cmd = static_cast<uint64_t>(Command::UPDATE);
                        msg.msg.off = ctx.outOff;
                        msg.msg.len = 0;
                        msg.msg.eof = true;
                        state = State::MSG_STORE;
                    }
                    else
                        state = State::READ_DATA;
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
                    logic.start(ctx.lastSize, ctx.compTime);
                    state = State::COMPUTE;
                }
                break;
            }
            case State::COMPUTE:
            {
                if(logic.handleMemResp(pkt, &delay))
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
                {
                    if (ctx.accSize >= ctx.reportSize ||
                        (ctx.inOff == ctx.dataSize && ctx.eof))
                        state = State::MSG_STORE;
                    else if (ctx.inOff == ctx.dataSize)
                        state = State::REPLY_SEND;
                    else
                        state = State::READ_DATA;
                }
                break;
            }

            case State::MSG_STORE:
            {
                state = State::MSG_SEND;
                break;
            }
            case State::MSG_SEND:
            {
                state = State::MSG_WAIT;
                break;
            }
            case State::MSG_WAIT:
            {
                Dtu::Command::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    auto err = static_cast<Dtu::Error>((long)cmd.error);
                    if (err == Dtu::Error::NONE)
                    {
                        if (ctx.inOff == ctx.dataSize)
                            state = State::REPLY_SEND;
                        else
                            state = State::READ_DATA;
                    }
                    else if (err == Dtu::Error::VPE_GONE)
                        state = State::MSG_ERROR;
                    else if (err == Dtu::Error::MISS_CREDITS)
                        state = State::MSG_IDLE;
                    else
                        state = State::MSG_SEND;
                }
                break;
            }
            case State::MSG_IDLE:
            {
                state = State::MSG_SEND;
                break;
            }
            case State::MSG_ERROR:
            {
                sysc.start(sizeof(msg));
                syscNext = State::READ_DATA;
                state = State::SYSCALL;
                break;
            }

            case State::REPLY_SEND:
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
                    state = cmd.error == 0 ? State::CTXSW
                                           : State::REPLY_ERROR;
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
DtuAccelStream::wakeup()
{
    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuAccelStream::interrupt()
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

    if (state != lastState)
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
            size_t left = ctx.dataSize - ctx.inOff;
            ctx.lastSize = std::min(bufSize, std::min(ctx.reportSize, left));
            pkt = createDtuCmdPkt(Dtu::Command::READ,
                                  EP_INPUT,
                                  BUF_ADDR,
                                  ctx.lastSize,
                                  ctx.off + ctx.inOff);
            ctx.inOff += ctx.lastSize;
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
            pkt = createDtuCmdPkt(Dtu::Command::WRITE,
                                  EP_OUTPUT,
                                  BUF_ADDR,
                                  ctx.lastSize,
                                  ctx.outOff);

            msg.msg.cmd = static_cast<uint64_t>(Command::UPDATE);
            msg.msg.off = ctx.outOff - ctx.accSize;
            msg.msg.len = ctx.accSize + ctx.lastSize;
            msg.msg.eof = ctx.eof && ctx.inOff == ctx.dataSize;

            ctx.accSize += ctx.lastSize;
            if (msg.msg.eof)
                ctx.outOff = 0;
            else
                ctx.outOff = (ctx.outOff + ctx.lastSize) % ctx.outSize;
            break;
        }
        case State::WRITE_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::MSG_STORE:
        {
            ctx.accSize = 0;
            pkt = createPacket(BUF_ADDR + bufSize,
                               sizeof(msg.msg),
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(), (char*)&msg.msg, sizeof(msg.msg));
            break;
        }
        case State::MSG_SEND:
        {
            pkt = createDtuCmdPkt(Dtu::Command::SEND,
                                  EP_SEND,
                                  BUF_ADDR + bufSize,
                                  sizeof(msg.msg),
                                  // specify an invalid reply EP
                                  0xFFFF);
            break;
        }
        case State::MSG_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::MSG_IDLE:
        {
            pkt = createDtuCmdPkt(Dtu::Command::SLEEP, 0, 0, 0, 0);
            break;
        }
        case State::MSG_ERROR:
        {
            msg.sys.opcode = 16;            /* FORWARD_MSG */
            msg.sys.sgate_sel = CAP_SGATE;
            msg.sys.rgate_sel = 0xFFFF;
            msg.sys.len = sizeof(msg.msg);
            msg.sys.rlabel = 0;
            msg.sys.event = 0;

            pkt = createPacket(MSG_ADDR, sizeof(msg), MemCmd::WriteReq);
            memcpy(pkt->getPtr<void>(), &msg, sizeof(msg));
            break;
        }

        case State::REPLY_SEND:
        {
            pkt = createDtuCmdPkt(Dtu::Command::REPLY,
                                  EP_RECV,
                                  BUF_ADDR + bufSize,
                                  1,
                                  ctx.msgAddr);
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
            reply.sys.opcode = 18;            /* FORWARD_REPLY */
            reply.sys.rgate_sel = CAP_RGATE;
            reply.sys.msgaddr = ctx.msgAddr;
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

DtuAccelStream*
DtuAccelStreamParams::create()
{
    return new DtuAccelStream(this);
}
