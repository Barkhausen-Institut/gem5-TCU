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

#include "cpu/tcu-accel-stream/accelerator.hh"
#include "debug/TcuAccelStream.hh"
#include "debug/TcuAccelStreamState.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/regfile.hh"
#include "sim/pe_memory.hh"

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
    "INOUT_ACK",

    "READ_DATA",
    "READ_DATA_WAIT",

    "WRITE_DATA",
    "WRITE_DATA_WAIT",

    "REPLY_STORE",
    "REPLY_SEND",
    "REPLY_WAIT",

    "CTXSW",

    "SYSCALL",

    "COMMIT_START",
    "COMMIT_SEND",
    "COMMIT_SEND_WAIT",

    "EXIT_ACK",
    "EXIT",
};

TcuAccelStream::TcuAccelStream(const TcuAccelStreamParams *p)
  : TcuAccel(p),
    irqPending(false),
    memPending(false),
    state(State::IDLE),
    lastState(State::FETCH_MSG), // something different
    lastFlags(),
    ctx(),
    bufSize(p->buf_size),
    sysc(this),
    yield(this),
    logic(p->logic),
    ctxsw(this),
    ctxSwPerformed()
{
    static_assert((sizeof(stateNames) / sizeof(stateNames[0]) ==
                  static_cast<size_t>(State::EXIT) + 1), "Missmatch");

    logic->setAccelerator(this);
}

std::string TcuAccelStream::getStateName() const
{
    std::ostringstream os;
    os << stateNames[static_cast<size_t>(state)];
    if (state == State::SYSCALL)
        os << ":" << sysc.stateName();
    else if (state == State::CTXSW)
        os << ":" << ctxsw.stateName();
    return os.str();
}

void
TcuAccelStream::completeRequest(PacketPtr pkt)
{
    RequestPtr req = pkt->req;

    if (ctx.flags != lastFlags || state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()))
    {
        DPRINTF(TcuAccelStreamState, "[%s:%#02x:in=%d,out=%d] Got response from memory\n",
            getStateName().c_str(), ctx.flags, ctx.inAvail, ctx.outAvail);
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
                    ctxSwPerformed = true;
                    ctx.flags &= ~Flags::FETCHED;
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
                CmdCommand::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    auto err = static_cast<TcuError>((long)cmd.error);
                    if (err == TcuError::NONE)
                        ctx.flags |= Flags::WAIT;
                    if (err == TcuError::INV_EP)
                    {
                        ctx.flags = Flags::EXIT;
                        state = State::EXIT;
                    }
                    else
                    {
                        // ignore other errors
                        state = State::IDLE;
                    }
                }
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
                if (regs[0] != static_cast<RegFile::reg_t>(-1))
                {
                    ctx.msgAddr = regs[0] + RBUF_ADDR;
                    DPRINTF(TcuAccelStream,
                            "Received message @ %p\n", ctx.msgAddr);
                    if (ctx.flags & Flags::EXIT)
                    {
                        (ctx.inReqAddr ? ctx.outReqAddr : ctx.inReqAddr) = ctx.msgAddr;
                        state = State::EXIT_ACK;
                    }
                    else
                        state = State::READ_MSG;
                }
                else if (ctx.inReqAddr && ctx.outAvail == 1)
                {
                    ctx.flags |= Flags::FETCHED;
                    ctx.msgAddr = ctx.inReqAddr;
                    state = State::READ_MSG;
                }
                else if (ctx.outReqAddr && ctx.inAvail == 0)
                {
                    ctx.flags |= Flags::FETCHED;
                    ctx.msgAddr = ctx.outReqAddr;
                    state = State::READ_MSG;
                }
                else
                {
                    ctx.flags |= Flags::FETCHED;
                    state = State::IDLE;
                }
                break;
            }
            case State::READ_MSG:
            {
                auto head = reinterpret_cast<const MessageHeader*>(pkt_data);
                auto args = reinterpret_cast<const uint64_t*>(
                    pkt_data + sizeof(MessageHeader));

                DPRINTF(TcuAccelStream,
                        "Considering message @ %p\n", ctx.msgAddr);

                switch(head->label)
                {
                    case LBL_IN_REPLY:
                    {
                        DPRINTF(
                            TcuAccelStream,
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
                            state = State::COMMIT_START;
                        }
                        else
                            state = State::INOUT_ACK;
                        break;
                    }

                    case LBL_OUT_REPLY:
                    {
                        DPRINTF(
                            TcuAccelStream,
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
                                TcuAccelStream,
                                "MSG: received input request(); state: (out=%d)\n",
                                ctx.outAvail
                            );
                        }

                        if (ctx.outAvail == 1)
                        {
                            reply.err = 0;
                            reply.off = 0;
                            reply.len = ctx.outLen;
                            ctx.outAvail = 0;
                            replyAddr = ctx.msgAddr;
                            replyNext = State::IDLE;
                            state = State::REPLY_STORE;
                            ctx.inReqAddr = 0;
                        }
                        else
                        {
                            state = State::IDLE;
                            ctx.inReqAddr = ctx.msgAddr;
                        }
                        break;
                    }

                    case LBL_OUT_REQ:
                    {
                        if (ctx.outReqAddr == 0)
                        {
                            DPRINTF(
                                TcuAccelStream,
                                "MSG: received output request(commit=%#llx); state: (in=%d)\n",
                                args[1], ctx.inAvail
                            );
                        }

                        if(!(ctx.flags & Flags::SEEN_EOF) &&
                           args[0] == static_cast<RegFile::reg_t>(Command::COMMIT))
                        {
                            ctx.flags |= Flags::SEEN_COMMIT;
                            // don't wait for the input reply; we won't get any
                            if(!(ctx.flags & Flags::OUTPUT))
                                ctx.flags &= ~Flags::WAIT;
                            ctx.commitOff = 0;
                            ctx.commitLen = args[1] == NO_COMMIT ? 0 : args[1];
                        }

                        if (ctx.inAvail == 0)
                        {
                            reply.err = 0;
                            reply.off = 0;
                            reply.len = bufSize;
                            replyAddr = ctx.msgAddr;
                            replyNext = State::IDLE;
                            ctx.inAvail = 1;
                            state = State::REPLY_STORE;
                            ctx.outReqAddr = 0;
                        }
                        else
                        {
                            state = State::IDLE;
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
                CmdCommand::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    ctx.bufOff = (cmd.error != 0) ? ctx.inOff : 0;
                    logic->start(ctx.bufOff, ctx.lastSize, Cycles(ctx.compTime));
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
                CmdCommand::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    if (ctx.lastSize == 0)
                    {
                        if (ctx.inPos == ctx.inLen)
                            ctx.inAvail = 0;
                        if (ctx.outPos == ctx.outLen)
                            ctx.outAvail = 1;
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
                CmdCommand::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                {
                    state = replyNext;
                }
                break;
            }

            case State::SYSCALL:
            {
                if(sysc.handleMemResp(pkt))
                    state = syscNext;
                break;
            }

            case State::COMMIT_START:
            {
                state = State::COMMIT_SEND;
                break;
            }
            case State::COMMIT_SEND:
            {
                state = State::COMMIT_SEND_WAIT;
                break;
            }
            case State::COMMIT_SEND_WAIT:
            {
                CmdCommand::Bits cmd =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if (cmd.opcode == 0)
                    state = State::EXIT_ACK;
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
TcuAccelStream::wakeup()
{
    ctx.flags &= ~Flags::FETCHED;
    sysc.retryFetch();

    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
TcuAccelStream::interrupt()
{
    // we'll handle the IRQ later
    irqPending = true;
}

void
TcuAccelStream::reset()
{
    irqPending = false;

    state = State::IDLE;
    memset(&ctx, 0, sizeof(ctx));
}

void
TcuAccelStream::logicFinished()
{
    ctx.lastSize = logic->outDataSize();
    ctx.flags &= ~(Flags::COMP | Flags::FETCHED);
    ctx.flags |= Flags::OUTPUT | Flags::COMPDONE;

    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
TcuAccelStream::tick()
{
    PacketPtr pkt = nullptr;

    // after a context switch, continue at then position we left off
    if (ctxSwPerformed)
    {
        ctxSwPerformed = false;

        // not started yet? just idle
        if (!(ctx.flags & Flags::STARTED))
            state = State::IDLE;
        else if (ctx.flags & Flags::INSYSC)
        {
            ctx.flags &= ~Flags::INSYSC;
            syscNext = static_cast<State>(ctx.nextSysc);
            sysc.start(0, true, true);
            state = State::SYSCALL;
        }
        else
            state = State::INOUT_START;
    }

    if (state == State::IDLE)
    {
        if (!(ctx.flags & Flags::WAIT) && (ctx.flags & Flags::COMPDONE))
        {
            state = State::INOUT_START;
            ctx.flags &= ~Flags::COMPDONE;
        }
        else if (ctx.flags & Flags::COMP)
            state = State::FETCH_MSG;
        else if (!(ctx.flags & Flags::WAIT) && (ctx.flags & Flags::TRANSFER))
        {
            state = State::INOUT_START;
            ctx.flags &= ~Flags::TRANSFER;
        }
        // can we answer a pending input request?
        else if (ctx.inReqAddr && ctx.outAvail == 1)
        {
            ctx.flags &= ~Flags::FETCHED;
            state = State::FETCH_MSG;
        }
        // can we answer a pending output request?
        else if (ctx.outReqAddr && ctx.inAvail == 0)
        {
            ctx.flags &= ~Flags::FETCHED;
            state = State::FETCH_MSG;
        }
        // if we're waiting for input and have seen a commit, give up waiting
        else if ((ctx.flags & (Flags::SEEN_COMMIT | Flags::OUTPUT)) == Flags::SEEN_COMMIT)
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
            // have seen commit? take that as input
            else if (ctx.flags & Flags::SEEN_COMMIT)
            {
                ctx.inOff = ctx.commitOff;
                ctx.inLen = ctx.commitLen;
                ctx.inPos = 0;
                ctx.flags &= ~Flags::SEEN_COMMIT;
                ctx.flags |= Flags::SEEN_EOF;
                state = State::READ_DATA;
            }
            // if we've seen EOF, commit and exit
            else if (ctx.flags & Flags::SEEN_EOF)
                state = State::COMMIT_START;
        }
        else
        {
            if (ctx.outPos < ctx.outLen)
                state = State::WRITE_DATA;
        }
    }

    if (ctx.flags != lastFlags || state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()))
    {
        DPRINTF(TcuAccelStreamState, "[%s:%#02x:in=%d,out=%d] tick\n",
            getStateName().c_str(), ctx.flags, ctx.inAvail, ctx.outAvail);
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
            rdwr_msg.cmd = static_cast<uint64_t>(
                (ctx.flags & Flags::OUTPUT) ? Command::NEXT_OUT : Command::NEXT_IN
            );
            rdwr_msg.commit = 0;

            DPRINTF(TcuAccelStream,
                    "MSG: sending %s request(commit=%#llx)\n",
                    (ctx.flags & Flags::OUTPUT) ? "output" : "input",
                    rdwr_msg.commit);

            pkt = createPacket(BUF_ADDR + bufSize,
                               sizeof(rdwr_msg),
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(),
                   (char*)&rdwr_msg,
                   sizeof(rdwr_msg));
            break;
        }
        case State::INOUT_SEND:
        {
            pkt = createTcuCmdPkt(
                CmdCommand::create(
                    CmdCommand::SEND,
                    (ctx.flags & Flags::OUTPUT) ? EP_OUT_SEND : EP_IN_SEND,
                    EP_RECV
                ),
                CmdData::create(BUF_ADDR + bufSize, sizeof(rdwr_msg)),
                (ctx.flags & Flags::OUTPUT) ? LBL_OUT_REPLY : LBL_IN_REPLY
            );
            break;
        }
        case State::INOUT_SEND_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::INOUT_ACK:
        {
            pkt = createTcuCmdPkt(
                CmdCommand::create(CmdCommand::ACK_MSG, EP_RECV,
                                   ctx.msgAddr - RBUF_ADDR),
                0
            );
            break;
        }

        case State::FETCH_MSG:
        {
            if (!(ctx.flags & Flags::FETCHED))
            {
                pkt = createTcuCmdPkt(
                    CmdCommand::create(CmdCommand::FETCH_MSG, EP_RECV),
                    0
                );
            }
            break;
        }
        case State::READ_MSG_ADDR:
        {
            Addr regAddr = getRegAddr(CmdReg::ARG1);
            pkt = createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::READ_MSG:
        {
            pkt = createPacket(ctx.msgAddr, MSG_SIZE, MemCmd::ReadReq);
            break;
        }

        case State::READ_DATA:
        {
            ctx.lastSize = std::min(bufSize, ctx.inLen - ctx.inPos);
            pkt = createTcuCmdPkt(
                CmdCommand::create(CmdCommand::READ, EP_IN_MEM,
                                   ctx.inOff + ctx.inPos),
                CmdData::create(BUF_ADDR, ctx.lastSize)
            );
            ctx.inPos += ctx.lastSize;
            break;
        }
        case State::READ_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::WRITE_DATA:
        {
            size_t amount = std::min(ctx.lastSize, ctx.outLen - ctx.outPos);
            pkt = createTcuCmdPkt(
                CmdCommand::create(CmdCommand::WRITE, EP_OUT_MEM,
                                   ctx.outOff + ctx.outPos),
                CmdData::create(BUF_ADDR + ctx.bufOff, amount)
            );

            ctx.bufOff += amount;
            ctx.lastSize -= amount;
            ctx.outPos += amount;
            break;
        }
        case State::WRITE_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::REPLY_STORE:
        {
            DPRINTF(TcuAccelStream,
                    "MSG: sending reply(off=%#llx, len=%#llx)\n",
                    reply.off, reply.len);

            pkt = createPacket(BUF_ADDR + bufSize,
                               sizeof(reply),
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(), (char*)&reply, sizeof(reply));
            break;
        }
        case State::REPLY_SEND:
        {
            pkt = createTcuCmdPkt(
                CmdCommand::create(CmdCommand::REPLY, EP_RECV,
                                   replyAddr - RBUF_ADDR),
                CmdData::create(BUF_ADDR + bufSize, sizeof(reply))
            );
            break;
        }
        case State::REPLY_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::SYSCALL:
        {
            pkt = sysc.tick();
            break;
        }

        case State::COMMIT_START:
        {
            rdwr_msg.cmd = static_cast<uint64_t>(Command::COMMIT);
            rdwr_msg.commit = ctx.outPos ? ctx.outPos : NO_COMMIT;

            DPRINTF(TcuAccelStream,
                    "MSG: sending commit request(nbytes=%#llx)\n",
                    rdwr_msg.commit);

            pkt = createPacket(BUF_ADDR + bufSize,
                               sizeof(rdwr_msg),
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(),
                   (char*)&rdwr_msg,
                   sizeof(rdwr_msg));
            break;
        }
        case State::COMMIT_SEND:
        {
            pkt = createTcuCmdPkt(
                CmdCommand::create(CmdCommand::SEND, EP_OUT_SEND, EP_RECV),
                CmdData::create(BUF_ADDR + bufSize, sizeof(rdwr_msg)),
                LBL_OUT_REPLY
            );
            break;
        }
        case State::COMMIT_SEND_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }

        case State::EXIT_ACK:
        {
            if (ctx.inReqAddr || ctx.outReqAddr)
            {
                auto addr = ctx.inReqAddr ? ctx.inReqAddr : ctx.outReqAddr;
                pkt = createTcuCmdPkt(
                    CmdCommand::create(CmdCommand::ACK_MSG,
                                       EP_RECV, addr - RBUF_ADDR),
                    0
                );
                break;
            }

            if (ctx.flags & Flags::EXIT)
            {
                state = State::IDLE;
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
            exit_msg.vpe_sel = SyscallSM::VPE_SEL;
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

TcuAccelStream*
TcuAccelStreamParams::create()
{
    return new TcuAccelStream(this);
}
