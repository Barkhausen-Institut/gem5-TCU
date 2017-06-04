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
static const unsigned CAP_RBUF      = 2;

static const size_t MSG_SIZE        = 64;
static const Addr BUF_ADDR          = 0x4000;

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
    "STORE_REPLY",
    "SEND_REPLY",
    "REPLY_WAIT",
    "REPLY_ERROR",

    "CTX_SAVE",
    "CTX_SAVE_DONE",
    "CTX_WAIT",

    "CTX_CHECK",
    "CTX_RESTORE",
    "CTX_RESTORE_WAIT",
    "CTX_RESTORE_READ",
    "CTX_RESTORE_DONE",

    "SYSCALL",
};

DtuAccelStream::DtuAccelStream(const DtuAccelStreamParams *p)
  : DtuAccel(p),
    algo(),
    irqPending(false),
    ctxSwPending(false),
    memPending(false),
    state(State::IDLE),
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

    DPRINTF(DtuAccelStreamState, "[%s] Got response from memory\n",
        getStateName().c_str());

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
                    {
                        irqPending = false;
                        state = State::CTX_SAVE;
                    }
                    else
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
                ctxSwPending = true;
                state = State::CTX_WAIT;
                break;
            }

            case State::CTX_CHECK:
            {
                uint64_t val = *pkt->getConstPtr<uint64_t>();
                if (val & RCTMuxCtrl::RESTORE)
                    state = State::CTX_RESTORE;
                else if ((val & RCTMuxCtrl::WAITING) && (~val & RCTMuxCtrl::STORE))
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
                        pkt_data + sizeof(Dtu::MessageHeader));

                DPRINTF(DtuAccelStream, "  inoff=%lld outoff=%lld len=%#llx auto=%d\n",
                    args[0], args[1], args[2], args[3]);

                autonomous = args[3];
                if (autonomous)
                {
                    inOff = args[0];
                    outOff = args[1];
                    dataOff = 0;
                    dataSize = args[2];
                    state = State::READ_DATA;
                }
                else
                {
                    lastSize = args[2];
                    streamOff = 0;
                    state = State::PULL_DATA;
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
                    delay = algo->getDelay(lastSize);
                    DPRINTF(DtuAccelStream, "%s for %luB took %llu cycles\n",
                        algo->name(), lastSize, delay);

                    if (delay > (curCycle() - opStart))
                        delay = delay - (curCycle() - opStart);
                    else
                        delay = Cycles(1);

                    if (!autonomous)
                    {
                        reply.msg.res = lastSize;
                        state = State::STORE_REPLY;
                    }
                    else
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
                    if (dataOff == dataSize || irqPending)
                    {
                        reply.msg.res = dataOff;
                        state = State::STORE_REPLY;
                    }
                    else
                        state = State::READ_DATA;
                }
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
                        state = State::CTX_CHECK;
                    else
                        state = State::REPLY_ERROR;
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
DtuAccelStream::interrupt()
{
    irqPending = true;
}

void
DtuAccelStream::wakeup()
{
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
    ctxSwPending = false;

    yield.start(false);
    state = State::IDLE;

    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuAccelStream::tick()
{
    PacketPtr pkt = nullptr;

    DPRINTF(DtuAccelStreamState, "[%s] tick\n",
        getStateName().c_str());

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
                state = State::CTX_SAVE;
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
            size_t left = dataSize - dataOff;
            lastSize = std::min(maxDataSize, left);
            pkt = createDtuCmdPkt(Dtu::Command::READ | (EP_INPUT << 4),
                                  BUF_ADDR,
                                  lastSize,
                                  inOff + dataOff);
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
            pkt = createDtuCmdPkt(Dtu::Command::WRITE | (EP_OUTPUT << 4),
                                  BUF_ADDR,
                                  lastSize,
                                  outOff + dataOff);
            dataOff += lastSize;
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
            pkt = createDtuCmdPkt(Dtu::Command::REPLY | (EP_RECV << 4),
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
            reply.sys.opcode = 18;          /* FORWARD_REPLY */
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

DtuAccelStream*
DtuAccelStreamParams::create()
{
    return new DtuAccelStream(this);
}
