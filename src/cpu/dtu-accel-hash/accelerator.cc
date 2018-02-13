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

#include "cpu/dtu-accel-hash/algorithm.hh"
#include "cpu/dtu-accel-hash/accelerator.hh"
#include "debug/DtuAccelHash.hh"
#include "debug/DtuAccelHashState.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/regfile.hh"
#include "sim/dtu_memory.hh"

#include <iomanip>

// the time for one 64 block; determined by ALADDIN and picking the sweet spot
// between area, power and performance. based on the SHA256 algorithm from:
// https://github.com/B-Con/crypto-algorithms/blob/master/sha256.c
static const Cycles BLOCK_TIME      = Cycles(85);
static const size_t BLOCK_SIZE      = 64;

static const char *stateNames[] =
{
    "IDLE",

    "FETCH_MSG",
    "READ_MSG_ADDR",
    "READ_MSG",
    "READ_DATA",
    "READ_DATA_WAIT",
    "HASH_DATA",
    "STORE_REPLY",
    "SEND_REPLY",
    "REPLY_WAIT",
    "REPLY_ERROR",

    "CTXSW",

    "SYSCALL",
};

DtuAccelHash::DtuAccelHash(const DtuAccelHashParams *p)
  : DtuAccel(p),
    bufSize(p->buf_size),
    irqPending(false),
    ctxSwPending(false),
    memPending(false),
    state(State::IDLE),
    lastState(State::CTXSW),    // something different
    hash(),
    msgAddr(),
    sysc(this),
    yield(this, &sysc),
    ctxsw(this),
    ctxSwPerformed()
{
    static_assert(sizeof(DtuAccelHashAlgorithm) % 64 == 0, "Hash state size invalid");

    yield.start();
}

std::string DtuAccelHash::getStateName() const
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
DtuAccelHash::completeRequest(PacketPtr pkt)
{
    Request* req = pkt->req;

    if (state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelHashState, "[%s] Got response from memory\n",
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
                    if (!ctxSwPerformed && hash.interrupted())
                        state = State::READ_DATA;
                    else
                        state = State::FETCH_MSG;
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
                    msgAddr = regs[0];
                    DPRINTF(DtuAccelHash, "Received message @ %p\n", msgAddr);
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

                DPRINTF(DtuAccelHash, "  cmd=%lld arg1=%#llx arg2=%#llx\n",
                    args[0], args[1], args[2]);

                replyOffset = 0;
                replySize = sizeof(uint64_t);
                switch(static_cast<Command>(args[0]))
                {
                    case Command::INIT:
                    {
                        bool autonomous = static_cast<bool>(args[1]);
                        auto algo = static_cast<DtuAccelHashAlgorithm::Type>(args[2]);
                        hash.start(autonomous, algo);

                        reply.msg.res = algo <= DtuAccelHashAlgorithm::SHA512;
                        state = State::STORE_REPLY;
                        break;
                    }

                    case Command::UPDATE:
                    {
                        if (hash.autonomous())
                        {
                            hash.startUpdate(args[1], args[2], 0);
                            state = State::READ_DATA;
                        }
                        else
                        {
                            hash.startUpdate(0, args[2], args[2]);
                            lastSize = args[2];
                            hashOff = 0;
                            state = State::HASH_DATA;
                        }
                        break;
                    }

                    case Command::FINISH:
                    {
                        reply.msg.res = hash.get(reply.msg.bytes);
                        assert(reply.msg.res <= sizeof(reply.msg.bytes));

                        std::ostringstream ss;
                        ss << std::hex;
                        for (size_t i = 0; i < reply.msg.res; i++)
                        {
                            ss << std::setw(2) << std::setfill('0')
                               << (int)reply.msg.bytes[i];
                        }
                        DPRINTF(DtuAccelHash, "Hash: %s\n", ss.str().c_str());

                        replySize += reply.msg.res;
                        state = State::STORE_REPLY;
                        break;
                    }
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
                    hashOff = 0;
                    state = State::HASH_DATA;
                }
                break;
            }
            case State::HASH_DATA:
            {
                hash.update(pkt->getPtr<char>(), pkt->getSize());

                hashOff += pkt->getSize();
                if (hashOff == lastSize)
                {
                    // the time for the hash operation on lastSize bytes
                    size_t blocks = (lastSize + BLOCK_SIZE - 1) / BLOCK_SIZE;
                    delay = Cycles(BLOCK_TIME * blocks);
                    DPRINTF(DtuAccelHash, "Hash for %luB took %llu cycles\n",
                        lastSize, delay);

                    // decrease it by the time we've already spent reading the
                    // data from SPM, because that's already included in the
                    // BLOCK_TIME.
                    // TODO if we use caches, this is not correct
                    if (delay > (curCycle() - hashStart))
                        delay = delay - (curCycle() - hashStart);
                    else
                        delay = Cycles(1);

                    if (hash.dataOffset() == hash.dataSize())
                    {
                        replyOffset = 0;
                        state = State::STORE_REPLY;
                    }
                    else if (hash.autonomous() && irqPending)
                    {
                        irqPending = false;
                        state = State::CTXSW;
                    }
                    else
                        state = State::READ_DATA;
                }
                break;
            }
            case State::STORE_REPLY:
            {
                replyOffset += pkt->getSize();
                if (replyOffset == replySize)
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
DtuAccelHash::interrupt()
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
DtuAccelHash::reset()
{
    irqPending = false;

    yield.start(false);
    state = State::IDLE;

    if (!memPending && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuAccelHash::tick()
{
    PacketPtr pkt = nullptr;

    // after a context switch, continue at then position we left off
    if (ctxSwPerformed)
    {
        state = hash.interrupted() ? State::READ_DATA : State::FETCH_MSG;
        ctxSwPerformed = false;
    }

    if (state != lastState ||
        (state == State::CTXSW && ctxsw.hasStateChanged()) ||
        (state == State::SYSCALL && sysc.hasStateChanged()) ||
        (state == State::IDLE && yield.hasStateChanged()))
    {
        DPRINTF(DtuAccelHashState, "[%s] tick\n",
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
        case State::READ_DATA:
        {
            size_t left = hash.dataSize() - hash.dataOffset();
            lastSize = std::min(maxDataSize, left);
            pkt = createDtuCmdPkt(Dtu::Command::READ,
                                  EP_DATA,
                                  BUF_ADDR,
                                  lastSize,
                                  hash.memOffset() + hash.dataOffset());
            hash.incOffset(lastSize);
            break;
        }
        case State::READ_DATA_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::HASH_DATA:
        {
            if (hashOff == 0)
                hashStart = curCycle();
            size_t rem = lastSize - hashOff;
            size_t size = std::min(chunkSize, rem);
            pkt = createPacket(BUF_ADDR + hashOff, size, MemCmd::ReadReq);
            break;
        }
        case State::STORE_REPLY:
        {
            size_t rem = replySize - replyOffset;
            size_t size = std::min(chunkSize, rem);
            pkt = createPacket(BUF_ADDR + replyOffset,
                               size,
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(), (char*)&reply.msg + replyOffset, size);
            break;
        }
        case State::SEND_REPLY:
        {
            pkt = createDtuCmdPkt(Dtu::Command::REPLY,
                                  EP_RECV,
                                  BUF_ADDR,
                                  replySize,
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
            reply.sys.len = replySize;
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

DtuAccelHash*
DtuAccelHashParams::create()
{
    return new DtuAccelHash(this);
}
