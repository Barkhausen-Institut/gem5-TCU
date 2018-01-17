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
#include "cpu/dtu-accel-stream/algorithm_fft.hh"
#include "cpu/dtu-accel-stream/algorithm_toupper.hh"
#include "debug/DtuAccelStream.hh"
#include "debug/DtuAccelStreamState.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/regfile.hh"
#include "sim/dtu_memory.hh"

#include <iomanip>

static const unsigned EP_RECV       = 7;
static const unsigned EP_INPUT      = 8;
static const unsigned EP_OUTPUT     = 9;
static const unsigned EP_SEND       = 10;
static const unsigned CAP_RGATE     = 2;
static const unsigned CAP_SGATE     = 3;

static const size_t MSG_SIZE        = 64;
static const Addr BUF_ADDR          = 0x6000;

static const char *stateNames[] =
{
    "IDLE",

    "FETCH_MSG",
    "READ_MSG_ADDR",
    "READ_MSG",
    "READ_DATA",
    "READ_DATA_WAIT",
    "PULL_DATA",
    "PUSH_DATA",
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

    "CTX_SAVE",
    "CTX_SAVE_DONE",
    "CTX_WAIT",

    "CTX_CHECK",
    "CTX_FLAGS",
    "CTX_RESTORE",

    "SYSCALL",
};

DtuAccelStream::DtuAccelStream(const DtuAccelStreamParams *p)
  : DtuAccel(p),
    algo(),
    irqPending(false),
    ctxSwPending(false),
    memPending(false),
    state(State::IDLE),
    lastState(State::CTX_CHECK), // something different
    msgAddr(),
    sysc(this),
    yield(this, &sysc)
{
    if (p->algorithm == 0)
        algo = new DtuAccelStreamAlgoFFT();
    else if(p->algorithm == 1)
        algo = new DtuAccelStreamAlgoToUpper();
    else
        panic("Unknown algorithm %d\n", p->algorithm);

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
                    state = State::CTX_CHECK;
                }
                break;
            }

            case State::CTX_WAIT:
            {
                assert(false);
                break;
            }

            case State::CTX_SAVE:
            {
                state = State::CTX_SAVE_DONE;
                break;
            }
            case State::CTX_SAVE_DONE:
            {
                state = State::CTX_WAIT;
                ctxSwPending = true;
                break;
            }

            case State::CTX_CHECK:
            {
                state = State::CTX_FLAGS;
                break;
            }
            case State::CTX_FLAGS:
            {
                uint64_t val = *pkt->getConstPtr<uint64_t>();
                if (val & RCTMuxCtrl::RESTORE)
                    state = State::CTX_RESTORE;
                else if (val & RCTMuxCtrl::STORE)
                    state = State::CTX_SAVE;
                else if (val & RCTMuxCtrl::WAITING)
                    state = State::CTX_RESTORE;
                else if (ctxSwPending)
                    state = State::CTX_WAIT;
                else
                    state = State::FETCH_MSG;
                break;
            }
            case State::CTX_RESTORE:
            {
                ctxSwPending = false;
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
                    DPRINTF(DtuAccelStream, "Received message @ %p\n", msgAddr);
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
                            "  init(bufsz=%#llx, outsz=%#llx,"
                            " reportsz=%#llx, comptime=%#llx)\n",
                            args[1], args[2], args[3], args[4]);

                    bufSize = args[1];
                    outSize = args[2];
                    reportSize = args[3];
                    compTime = Cycles(args[4]);
                    accSize = 0;
                    outOff = 0;
                    state = State::REPLY_SEND;
                }
                else
                {
                    DPRINTF(DtuAccelStream,
                            "  update(off=%lld len=%#llx eof=%lld)\n",
                            args[1], args[2], args[3]);

                    off = args[1];
                    inOff = 0;
                    dataSize = args[2];
                    eof = args[3];
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
                    streamOff = 0;
                    state = State::PULL_DATA;
                }
                break;
            }
            case State::PULL_DATA:
            {
                // load data to be able to change it and write it back
                lastData = new uint8_t[lastPullSize];

                // execute the algorithm
                algo->execute(lastData,
                              pkt->getConstPtr<uint8_t>(),
                              lastPullSize);

                state = State::PUSH_DATA;
                break;
            }
            case State::PUSH_DATA:
            {
                streamOff += lastPullSize;

                if (streamOff == lastSize)
                {
                    // decrease it by the time we've already spent reading the
                    // data from SPM, because that's already included in the
                    // BLOCK_TIME.
                    // TODO if we use caches, this is not correct
                    delay = algo->getDelay(compTime, lastSize);
                    DPRINTF(DtuAccelStream, "%s for %luB took %llu cycles\n",
                        algo->name(), lastSize, delay);

                    if (delay > (curCycle() - opStart))
                        delay = delay - (curCycle() - opStart);
                    else
                        delay = Cycles(1);

                    state = State::WRITE_DATA;
                }
                else
                    state = State::PULL_DATA;
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
                    if (accSize >= reportSize || (inOff == dataSize && eof))
                        state = State::MSG_STORE;
                    else if (inOff == dataSize)
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
                        if (inOff == dataSize)
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
                    state = cmd.error == 0 ? State::CTX_CHECK
                                           : State::REPLY_ERROR;
                }
                break;
            }
            case State::REPLY_ERROR:
            {
                sysc.start(sizeof(reply));
                syscNext = State::CTX_CHECK;
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

    if (state == State::CTX_WAIT)
    {
        state = State::CTX_CHECK;
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

        case State::CTX_WAIT:
            break;

        case State::CTX_SAVE:
        {
            Addr regAddr = getRegAddr(CmdReg::ABORT);
            uint64_t value = Dtu::Command::ABORT_VPE;
            pkt = createDtuRegPkt(regAddr, value, MemCmd::WriteReq);
            break;
        }
        case State::CTX_SAVE_DONE:
        {
            pkt = createPacket(RCTMUX_FLAGS, sizeof(uint64_t), MemCmd::WriteReq);
            *pkt->getPtr<uint64_t>() = RCTMuxCtrl::SIGNAL;
            break;
        }

        case State::CTX_CHECK:
        {
            Addr regAddr = getRegAddr(ReqReg::EXT_REQ);
            pkt = createDtuRegPkt(regAddr, sizeof(uint64_t), MemCmd::WriteReq);
            *pkt->getPtr<uint64_t>() = 0;
            break;
        }
        case State::CTX_FLAGS:
        {
            pkt = createPacket(RCTMUX_FLAGS, sizeof(uint64_t), MemCmd::ReadReq);
            break;
        }
        case State::CTX_RESTORE:
        {
            pkt = createPacket(RCTMUX_FLAGS, sizeof(uint64_t), MemCmd::WriteReq);
            *pkt->getPtr<uint64_t>() = RCTMuxCtrl::SIGNAL;
            break;
        }

        case State::FETCH_MSG:
        {
            if (irqPending)
            {
                irqPending = false;
                state = State::CTX_CHECK;
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
        case State::READ_DATA:
        {
            size_t left = dataSize - inOff;
            lastSize = std::min(bufSize, std::min(reportSize, left));
            pkt = createDtuCmdPkt(Dtu::Command::READ,
                                  EP_INPUT,
                                  BUF_ADDR,
                                  lastSize,
                                  off + inOff);
            inOff += lastSize;
            break;
        }
        case State::READ_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::PULL_DATA:
        {
            if (streamOff == 0)
                opStart = curCycle();
            size_t rem = lastSize - streamOff;
            lastPullSize = std::min(chunkSize, rem);
            pkt = createPacket(BUF_ADDR + streamOff, lastPullSize, MemCmd::ReadReq);
            break;
        }
        case State::PUSH_DATA:
        {
            pkt = createPacket(BUF_ADDR + streamOff,
                               lastData,
                               lastPullSize,
                               MemCmd::WriteReq);
            break;
        }
        case State::WRITE_DATA:
        {
            pkt = createDtuCmdPkt(Dtu::Command::WRITE,
                                  EP_OUTPUT,
                                  BUF_ADDR,
                                  lastSize,
                                  outOff);

            msg.msg.cmd = static_cast<uint64_t>(Command::UPDATE);
            msg.msg.off = outOff - accSize;
            msg.msg.len = accSize + lastSize;
            msg.msg.eof = eof && inOff == dataSize;

            accSize += lastSize;
            if (msg.msg.eof)
                outOff = 0;
            else
                outOff = (outOff + lastSize) % outSize;
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
            accSize = 0;
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
            reply.sys.opcode = 18;            /* FORWARD_REPLY */
            reply.sys.rgate_sel = CAP_RGATE;
            reply.sys.msgaddr = msgAddr;
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
