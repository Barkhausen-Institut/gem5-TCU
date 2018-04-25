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

#include "cpu/dtu-accel-aladdin/accelerator.hh"
#include "debug/DtuAccelAladdin.hh"
#include "debug/DtuAccelAladdinState.hh"
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
    "COMPUTE",
    "STORE_REPLY",
    "SEND_REPLY",
    "REPLY_WAIT",
    "REPLY_ERROR",

    "CTXSW",

    "SYSCALL",
};

DtuAccelAladdin::DtuAccelAladdin(const DtuAccelAladdinParams *p)
  : DtuAccel(p),
    irqPending(false),
    ctxSwPending(false),
    memPending(false),
    state(State::IDLE),
    lastState(State::CTXSW),    // something different
    ctx(),
    accelId(p->accel_id),
    sysc(this),
    yield(this, &sysc),
    ctxsw(this),
    ctxSwPerformed()
{
    static_assert((sizeof(stateNames) / sizeof(stateNames[0]) ==
                  static_cast<size_t>(State::SYSCALL) + 1), "Missmatch");

    yield.start();
}

std::string DtuAccelAladdin::getStateName() const
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
DtuAccelAladdin::completeRequest(PacketPtr pkt)
{
    Request* req = pkt->req;

    if (state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelAladdinState, "[%s] Got response from memory\n",
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
                    ctx.msgOff = 0;
                    DPRINTF(DtuAccelAladdin, "Received message @ %p\n", ctx.msgAddr);
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
                size_t maxOff = sizeof(ctx.msg) + sizeof(MessageHeader);
                if (ctx.msgOff < maxOff)
                {
                    char *dst = (char*)&ctx.msg;
                    size_t amount = std::min<size_t>(maxOff - ctx.msgOff, pkt->getSize());
                    if (ctx.msgOff == 0)
                    {
                        pkt_data += sizeof(MessageHeader);
                        amount -= sizeof(MessageHeader);
                    }
                    else
                        dst += ctx.msgOff - sizeof(MessageHeader);
                    memcpy(dst, pkt_data, amount);
                }

                ctx.msgOff += pkt->getSize();
                if (ctx.msgOff == MSG_SIZE)
                {
                    DPRINTF(DtuAccelAladdin,
                        "  invokeMsg(iters=%llu, arrays=%llu: [\n",
                        ctx.msg.iterations, ctx.msg.array_count);
                    for(uint64_t i = 0; i < ctx.msg.array_count; ++i)
                    {
                        DPRINTFR(DtuAccelAladdin, "    addr=%#llx, size=%#llx\n",
                            ctx.msg.arrays[i].addr, ctx.msg.arrays[i].size);
                    }
                    DPRINTFR(DtuAccelAladdin, "  ])\n");

                    ctx.iteration = 0;
                    ctx.interrupted = false;
                    state = State::COMPUTE;
                }
                break;
            }
            case State::COMPUTE:
            {
                assert(false);
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
DtuAccelAladdin::interrupt()
{
    irqPending = true;

    if (ctxsw.isWaiting())
    {
        ctxsw.restart();
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
    else if(state == State::COMPUTE)
    {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void
DtuAccelAladdin::wakeup()
{
    sysc.retryFetch();
}

void
DtuAccelAladdin::reset()
{
    irqPending = false;

    yield.start(false);
    state = State::IDLE;

    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuAccelAladdin::signalFinished(size_t off)
{
    ctx.trace_off = off;
    ctx.iteration += 1;

    assert(state == State::COMPUTE);
    if (ctx.iteration == ctx.msg.iterations)
        state = State::STORE_REPLY;

    if (!tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuAccelAladdin::tick()
{
    PacketPtr pkt = nullptr;

    // after a context switch, continue at then position we left off
    if (ctxSwPerformed)
    {
        DPRINTF(DtuAccelAladdin,
            "Resuming %swork at %llu/%llu, off=%llu\n",
            ctx.interrupted ? "interrupted " : "",
            ctx.iteration, ctx.msg.iterations, ctx.trace_off);

        state = ctx.interrupted ? State::COMPUTE : State::FETCH_MSG;
        ctxSwPerformed = false;
        ctx.interrupted = false;
        irqPending = false;
    }

    if (state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelAladdinState, "[%s] tick\n",
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
            size_t size = std::min(chunkSize, MSG_SIZE - ctx.msgOff);
            pkt = createPacket(ctx.msgAddr + ctx.msgOff, size, MemCmd::ReadReq);
            break;
        }

        case State::COMPUTE:
        {
            if (irqPending)
            {
                DPRINTF(DtuAccelAladdin,
                    "Interrupting work at %llu/%llu, off=%llu\n",
                    ctx.iteration, ctx.msg.iterations, ctx.trace_off);

                irqPending = false;
                ctx.interrupted = true;
                system->resetAccelerator(accelId);
                state = State::CTXSW;
                schedule(tickEvent, clockEdge(Cycles(1)));
                break;
            }

            auto addArray = [this](const char *name, Addr addr, Addr size) {
                system->insertArrayLabelMapping(
                    accelId, name, addr, size
                );
                // establish 1:1 mapping
                Addr page_off = addr & (TheISA::PageBytes - 1);
                int num_pages = ceil(((float)size + page_off) / TheISA::PageBytes);
                for(int i = 0; i < num_pages; i++)
                {
                    system->insertAddressTranslationMapping(
                        accelId,
                        addr + i * TheISA::PageBytes,
                        addr + i * TheISA::PageBytes
                    );
                }
            };

            // store the parameter names here instead of letting the client
            // send them, because we don't want to pay for the performance
            // overhead (they wouldn't be there in real life)
            static const char *stencil_arrays[] = {
                "orig", "sol", "C"
            };
            static const char *md_arrays[] = {
                "position_x", "position_y", "position_z",
                "force_x", "force_y", "force_z", "NL"
            };
            static const char *spmv_arrays[] = {
                "val", "cols", "rowDelimiters", "vec", "out"
            };
            static const char *fft_arrays[] = {
                "work_x", "work_y"
            };

            const char **names = nullptr;
            if (accelId == 0x00000120)
                names = stencil_arrays;
            else if (accelId == 0x000000B0)
                names = md_arrays;
            else if (accelId == 0x000000F0)
                names = spmv_arrays;
            else if (accelId == 0x00000060)
                names = fft_arrays;
            else
                fatal("Unknown accelerator id %d", accelId);

            for (uint64_t i = 0; i < ctx.msg.array_count; ++i)
                addArray(names[i], ctx.msg.arrays[i].addr, ctx.msg.arrays[i].size);
            system->activateAccelerator(accelId, 0x1000, 0, 0, ctx.trace_off);

            pkt = nullptr;
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
            reply.sys.opcode = SyscallSM::Operation::FORWARD_REPLY;
            reply.sys.cap = CAP_RBUF;
            reply.sys.msgaddr = ctx.msgAddr;
            reply.sys.event = 0;
            reply.sys.len = sizeof(reply.msg);
            reply.sys.event = 0;

            pkt = createPacket(BUF_ADDR, sizeof(reply), MemCmd::WriteReq);
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

DtuAccelAladdin*
DtuAccelAladdinParams::create()
{
    return new DtuAccelAladdin(this);
}
