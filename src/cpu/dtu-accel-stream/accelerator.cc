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

    "WRITE_DATA",
    "WRITE_DATA_WAIT",

    "REPLY_STORE",
    "REPLY_SEND",
    "REPLY_WAIT",
    "REPLY_ERROR",

    "CTXSW",

    "SYSCALL",

    "SUBMIT_START",
    "SUBMIT_SEND",
    "SUBMIT_SEND_WAIT",
    "SUBMIT_SEND_ERROR",

    "EXIT_ACK",
    "EXIT",
};

DtuAccelStream::DtuAccelStream(const DtuAccelStreamParams *p)
  : DtuAccel(p),
    irqPending(false),
    memPending(false),
    state(State::IDLE),
    lastState(State::FETCH_MSG), // something different
    lastFlags(),
    bufOff(),
    ctx(),
    bufSize(p->buf_size),
    sysc(this),
    yield(this, &sysc),
    logic(p->logic),
    ctxsw(this),
    ctxSwPerformed()
{
    static_assert((sizeof(stateNames) / sizeof(stateNames[0]) ==
                  static_cast<size_t>(State::EXIT) + 1), "Missmatch");

    rdwr_msg.sys.opcode = SyscallSM::Operation::FORWARD_MSG;
    rdwr_msg.sys.rgate_sel = CAP_RECV;
    rdwr_msg.sys.len = sizeof(rdwr_msg.msg);
    rdwr_msg.sys.event = 0;

    logic->setAccelerator(this);
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
    else if (state == State::CTXSW)
        os << ":" << ctxsw.stateName();
    return os.str();
}

void
DtuAccelStream::completeRequest(PacketPtr pkt)
{
    Request* req = pkt->req;

    if (ctx.flags != lastFlags || state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelStreamState, "[%s:%#02x:in=%#02x,out=%#02x] Got response from memory\n",
            getStateName().c_str(), ctx.flags, ctx.inMask, ctx.outMask);
        lastState = state;
        lastFlags = ctx.flags;
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
                    ctx.flags &= ~Flags::FETCHED;
                    state = (ctx.flags & Flags::WAIT) ? State::FETCH_MSG
                                                      : State::INOUT_START;
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
                    {
                        ctx.flags |= Flags::WAIT;
                        state = State::INOUT_SEND_ERROR;
                    }
                    else
                    {
                        if (err == Dtu::Error::NONE)
                            ctx.flags |= Flags::WAIT;
                        if (err == Dtu::Error::INV_EP)
                        {
                            ctx.flags = Flags::EXIT;
                            state = State::EXIT;
                        }
                        else
                        {
                            // ignore other errors
                            yield.start();
                            state = State::IDLE;
                        }
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
                state = (ctx.flags & Flags::OUTPUT) ? State::WRITE_DATA
                                                    : State::READ_DATA;
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
                if (regs[0])
                {
                    ctx.msgAddr = regs[0];
                    DPRINTF(DtuAccelStream,
                            "Received message @ %p\n", ctx.msgAddr);
                    if (ctx.flags & Flags::EXIT)
                    {
                        (ctx.inReqAddr ? ctx.outReqAddr : ctx.inReqAddr) = ctx.msgAddr;
                        state = State::EXIT_ACK;
                    }
                    else
                        state = State::READ_MSG;
                }
                else if (ctx.inReqAddr || ctx.outReqAddr)
                {
                    ctx.flags |= Flags::FETCHED;
                    ctx.msgAddr = ctx.inReqAddr ? ctx.inReqAddr : ctx.outReqAddr;
                    state = State::READ_MSG;
                }
                else
                {
                    ctx.flags |= Flags::FETCHED;
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

                DPRINTF(DtuAccelStream,
                        "Considering message @ %p\n", ctx.msgAddr);

                switch(head->label)
                {
                    case LBL_IN_REPLY:
                    {
                        DPRINTF(
                            DtuAccelStream,
                            "MSG: received input reply(err=%#llx, off=%#llx, len=%#llx)\n",
                            args[0], args[1], args[2]
                        );

                        ctx.inOff = args[1];
                        ctx.inLen = args[2];
                        ctx.inPos = 0;
                        ctx.flags &= ~Flags::WAIT;
                        if (ctx.inLen == 0)
                        {
                            ctx.flags |= Flags::SEEN_EOF;
                            state = State::SUBMIT_START;
                        }
                        else
                            state = State::INOUT_ACK;
                        break;
                    }

                    case LBL_OUT_REPLY:
                    {
                        DPRINTF(
                            DtuAccelStream,
                            "MSG: received output reply(err=%#llx, off=%#llx, len=%#llx)\n",
                            args[0], args[1], args[2]
                        );

                        ctx.outOff = args[1];
                        ctx.outLen = args[2];
                        ctx.outPos = 0;
                        ctx.flags &= ~Flags::WAIT;
                        state = State::INOUT_ACK;
                        break;
                    }

                    case LBL_IN_REQ:
                    {
                        if (ctx.inReqAddr == 0)
                        {
                            DPRINTF(
                                DtuAccelStream,
                                "MSG: received input request();"
                                " state: (outMask=%#02x)\n",
                                ctx.outMask
                            );
                        }

                        if (ctx.outMask != 0b00)
                        {
                            reply.msg.err = 0;
                            if (ctx.outMask & 0b01)
                            {
                                reply.msg.off = 0;
                                reply.msg.len = ctx.outMaskLen[0];
                                ctx.outMask &= ~0b01;
                            }
                            else
                            {
                                reply.msg.off = bufSize / 2;
                                reply.msg.len = ctx.outMaskLen[1];
                                ctx.outMask &= ~0b10;
                            }
                            replyAddr = ctx.msgAddr;
                            replyNext = State::IDLE;
                            state = State::REPLY_STORE;
                            ctx.inReqAddr = 0;
                        }
                        else
                        {
                            state = State::IDLE;
                            yield.start();
                            ctx.inReqAddr = ctx.msgAddr;
                        }
                        break;
                    }

                    case LBL_OUT_REQ:
                    {
                        if (ctx.outReqAddr == 0)
                        {
                            DPRINTF(
                                DtuAccelStream,
                                "MSG: received output request(submit=%#llx);"
                                " state: (inMask=%#02x)\n",
                                args[1], ctx.inMask
                            );
                        }

                        // submit != 0 implies EOF
                        if (args[1] != 0)
                        {
                            ctx.flags |= Flags::SEEN_SUBMIT;
                            // don't wait for the input reply; we won't get any
                            if(!(ctx.flags & Flags::OUTPUT))
                                ctx.flags &= ~Flags::WAIT;
                            ctx.commitOff = 0;
                            ctx.commitLen = args[1] == NO_SUBMIT ? 0 : args[1];
                        }

                        if (ctx.inMask != 0b11)
                        {
                            reply.msg.err = 0;
                            reply.msg.off = !(ctx.inMask & 0b01) ? 0x0 : bufSize / 2;
                            reply.msg.len = bufSize / 2;
                            replyAddr = ctx.msgAddr;
                            replyNext = State::IDLE;
                            ctx.inMask |= !(ctx.inMask & 0b01) ? 0b01 : 0b10;
                            state = State::REPLY_STORE;
                            ctx.outReqAddr = 0;
                        }
                        else
                        {
                            state = State::IDLE;
                            yield.start();
                            ctx.outReqAddr = ctx.msgAddr;
                        }
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
                    bufOff = (cmd.error != 0) ? ctx.inOff : 0;
                    logic->start(bufOff, ctx.lastSize, Cycles(ctx.compTime));
                    ctx.flags |= Flags::COMP;
                    state = State::FETCH_MSG;
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
                    if (ctx.lastSize == 0)
                    {
                        if (ctx.inPos == ctx.inLen)
                            ctx.inMask &= ~((ctx.inOff == 0) ? 0b01 : 0b10);
                        if (ctx.outPos == ctx.outLen)
                        {
                            ctx.outMaskLen[ctx.outOff == 0 ? 0 : 1] = ctx.outLen;
                            ctx.outMask |= ctx.outOff == 0 ? 0b01 : 0b10;
                        }
                        ctx.flags &= ~Flags::OUTPUT;
                    }
                    state = State::INOUT_START;
                }
                break;
            }

            case State::REPLY_STORE:
            {
                state = State::REPLY_SEND;
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
                    state = cmd.error == 0 ? replyNext
                                           : State::REPLY_ERROR;
                }
                break;
            }
            case State::REPLY_ERROR:
            {
                sysc.start(sizeof(reply));
                syscNext = replyNext;
                state = State::SYSCALL;
                break;
            }

            case State::SYSCALL:
            {
                if(sysc.handleMemResp(pkt))
                    state = syscNext;
                break;
            }

            case State::SUBMIT_START:
            {
                state = State::SUBMIT_SEND;
                break;
            }
            case State::SUBMIT_SEND:
            {
                state = State::SUBMIT_SEND_WAIT;
                break;
            }
            case State::SUBMIT_SEND_WAIT:
            {
                Dtu::Command::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    auto err = static_cast<Dtu::Error>((long)cmd.error);
                    if (err == Dtu::Error::VPE_GONE)
                        state = State::SUBMIT_SEND_ERROR;
                    else
                        state = State::EXIT_ACK;
                }
                break;
            }
            case State::SUBMIT_SEND_ERROR:
            {
                sysc.start(sizeof(rdwr_msg));
                syscNext = State::EXIT_ACK;
                state = State::SYSCALL;
                break;
            }

            case State::EXIT_ACK:
            {
                if (ctx.inReqAddr)
                    ctx.inReqAddr = 0;
                else
                    ctx.outReqAddr = 0;
                break;
            }

            case State::EXIT:
            {
                sysc.start(sizeof(exit_msg), false);
                syscNext = State::IDLE;
                state = State::SYSCALL;
                ctx.flags |= Flags::EXIT;
                yield.start();
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
    ctx.flags &= ~Flags::FETCHED;
    sysc.retryFetch();

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
DtuAccelStream::logicFinished()
{
    ctx.lastSize = logic->outDataSize();
    ctx.flags &= ~(Flags::COMP | Flags::FETCHED);
    ctx.flags |= Flags::OUTPUT | Flags::COMPDONE;

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
        if (ctx.flags & Flags::INSYSC)
        {
            ctx.flags &= ~Flags::INSYSC;
            syscNext = static_cast<State>(ctx.nextSysc);
            sysc.start(0, true, true);
            state = State::SYSCALL;
        }
        else
            state = State::INOUT_START;
        ctxSwPerformed = false;
    }

    if (state == State::IDLE)
    {
        if (!(ctx.flags & Flags::WAIT) && ctx.flags & Flags::COMPDONE)
        {
            state = State::INOUT_START;
            ctx.flags &= ~Flags::COMPDONE;
        }
        else if (ctx.flags & Flags::COMP)
            state = State::FETCH_MSG;
        else if (!(ctx.flags & Flags::WAIT) && ctx.flags & Flags::TRANSFER)
        {
            state = State::INOUT_START;
            ctx.flags &= ~Flags::TRANSFER;
        }
        // can we answer a pending input request?
        else if (ctx.inReqAddr && ctx.outMask != 0b00)
        {
            ctx.flags &= ~Flags::FETCHED;
            state = State::FETCH_MSG;
        }
        // can we answer a pending output request?
        else if (ctx.outReqAddr && ctx.inMask != 0b11)
        {
            ctx.flags &= ~Flags::FETCHED;
            state = State::FETCH_MSG;
        }
        // if we're waiting for input and have seen a submit, give up waiting
        else if ((ctx.flags & (Flags::SEEN_SUBMIT | Flags::OUTPUT)) == Flags::SEEN_SUBMIT)
            state = State::INOUT_START;
    }

    if (state == State::SYSCALL && irqPending &&
        sysc.isWaiting() && !(ctx.flags & Flags::COMP))
    {
        irqPending = false;
        ctx.flags |= Flags::INSYSC;
        ctx.nextSysc = static_cast<uint64_t>(syscNext);
        state = State::CTXSW;
    }

    if (state == State::INOUT_START)
    {
        if (irqPending)
        {
            irqPending = false;
            state = State::CTXSW;
        }
        // if we waked up because of some message, just ack it
        // alternatively, if we are already waiting for a response, just check
        else if (ctx.flags & (Flags::EXIT | Flags::WAIT))
            state = State::FETCH_MSG;
        else if (!(ctx.flags & Flags::OUTPUT))
        {
            // should we check for messages in between?
            if (!(ctx.flags & Flags::FETCHED))
            {
                ctx.flags |= Flags::TRANSFER;
                state = State::FETCH_MSG;
            }
            // can we still read?
            else if (ctx.inPos < ctx.inLen)
                state = State::READ_DATA;
            // have seen submit? take that as input
            else if (ctx.flags & Flags::SEEN_SUBMIT)
            {
                ctx.inOff = ctx.commitOff;
                ctx.inLen = ctx.commitLen;
                ctx.inPos = 0;
                ctx.flags &= ~Flags::SEEN_SUBMIT;
                ctx.flags |= Flags::SEEN_EOF;
                state = State::READ_DATA;
            }
            // if we've seen EOF, submit and exit
            else if (ctx.flags & Flags::SEEN_EOF)
                state = State::SUBMIT_START;
        }
        else
        {
            if (ctx.outPos < ctx.outLen)
                state = State::WRITE_DATA;
        }
    }

    if (ctx.flags != lastFlags || state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelStreamState, "[%s:%#02x:in=%#02x,out=%#02x] tick\n",
            getStateName().c_str(), ctx.flags, ctx.inMask, ctx.outMask);
        lastState = state;
        lastFlags = ctx.flags;
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
                (ctx.flags & Flags::OUTPUT) ? Command::WRITE : Command::READ
            );
            rdwr_msg.msg.submit = 0;

            DPRINTF(DtuAccelStream,
                    "MSG: sending %s request(submit=%#llx)\n",
                    (ctx.flags & Flags::OUTPUT) ? "output" : "input",
                    rdwr_msg.msg.submit);

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
                (ctx.flags & Flags::OUTPUT) ? EP_OUT_SEND : EP_IN_SEND,
                BUF_ADDR + bufSize,
                sizeof(rdwr_msg.msg),
                EP_RECV,
                (ctx.flags & Flags::OUTPUT) ? LBL_OUT_REPLY : LBL_IN_REPLY
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
            rdwr_msg.sys.sgate_sel = (ctx.flags & Flags::OUTPUT) ? CAP_OUT : CAP_IN;
            rdwr_msg.sys.rlabel = (ctx.flags & Flags::OUTPUT) ? LBL_OUT_REPLY : LBL_IN_REPLY;
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
            if (!(ctx.flags & Flags::FETCHED))
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
            ctx.lastSize = std::min(bufSize / 8, ctx.inLen - ctx.inPos);
            pkt = createDtuCmdPkt(Dtu::Command::READ,
                                  EP_IN_MEM,
                                  BUF_ADDR,
                                  ctx.lastSize,
                                  ctx.inOff + ctx.inPos);
            ctx.inPos += ctx.lastSize;
            break;
        }
        case State::READ_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::WRITE_DATA:
        {
            size_t amount = std::min(ctx.lastSize, ctx.outLen - ctx.outPos);
            pkt = createDtuCmdPkt(Dtu::Command::WRITE,
                                  EP_OUT_MEM,
                                  BUF_ADDR + bufOff,
                                  amount,
                                  ctx.outOff + ctx.outPos);

            bufOff += amount;
            ctx.lastSize -= amount;
            ctx.outPos += amount;
            break;
        }
        case State::WRITE_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::REPLY_STORE:
        {
            DPRINTF(DtuAccelStream,
                    "MSG: sending reply(off=%#llx, len=%#llx)\n",
                    reply.msg.off, reply.msg.len);

            pkt = createPacket(BUF_ADDR + bufSize,
                               sizeof(reply.msg),
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(), (char*)&reply.msg, sizeof(reply.msg));
            break;
        }
        case State::REPLY_SEND:
        {
            pkt = createDtuCmdPkt(Dtu::Command::REPLY,
                                  EP_RECV,
                                  BUF_ADDR + bufSize,
                                  sizeof(reply.msg),
                                  replyAddr);
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
            reply.sys.rgate_sel = CAP_RECV;
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

        case State::SUBMIT_START:
        {
            rdwr_msg.msg.cmd = static_cast<uint64_t>(Command::WRITE);
            rdwr_msg.msg.submit = ctx.outPos ? ctx.outPos : NO_SUBMIT;

            DPRINTF(DtuAccelStream,
                    "MSG: sending output request(submit=%#llx)\n",
                    rdwr_msg.msg.submit);

            pkt = createPacket(BUF_ADDR + bufSize,
                               sizeof(rdwr_msg.msg),
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(),
                   (char*)&rdwr_msg.msg,
                   sizeof(rdwr_msg.msg));
            break;
        }
        case State::SUBMIT_SEND:
        {
            pkt = createDtuCmdPkt(
                Dtu::Command::SEND,
                EP_OUT_SEND,
                BUF_ADDR + bufSize,
                sizeof(rdwr_msg.msg),
                EP_RECV,
                LBL_OUT_REPLY
            );
            break;
        }
        case State::SUBMIT_SEND_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::SUBMIT_SEND_ERROR:
        {
            rdwr_msg.sys.sgate_sel = CAP_OUT;
            rdwr_msg.sys.rlabel = LBL_OUT_REPLY;
            pkt = createPacket(MSG_ADDR, sizeof(rdwr_msg), MemCmd::WriteReq);
            memcpy(pkt->getPtr<void>(), &rdwr_msg, sizeof(rdwr_msg));
            break;
        }

        case State::EXIT_ACK:
        {
            if (ctx.inReqAddr || ctx.outReqAddr)
            {
                auto addr = ctx.inReqAddr ? ctx.inReqAddr : ctx.outReqAddr;
                pkt = createDtuCmdPkt(Dtu::Command::ACK_MSG,
                                      EP_RECV,
                                      0,
                                      0,
                                      addr);
                break;
            }

            if (ctx.flags & Flags::EXIT)
            {
                state = State::IDLE;
                yield.start();
                pkt = yield.tick();
                break;
            }
            else
            {
                state = State::EXIT;
                [[fallthrough]];
            }
        }

        case State::EXIT:
        {
            exit_msg.opcode = SyscallSM::Operation::VPE_CTRL;
            exit_msg.op = SyscallSM::VPEOp::VCTRL_STOP;
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
