/*
 * Copyright (c) 2016, Nils Asmussen
 * Copyright (c) 2015, Christian Menard
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
#include "debug/DtuAccel.hh"
#include "debug/DtuAccelAccess.hh"
#include "debug/DtuAccelState.hh"
#include "debug/DtuConnector.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/regfile.hh"
#include "sim/dtu_memory.hh"

#include <iomanip>

static const unsigned EP_SYSS       = 0;
static const unsigned EP_SYSR       = 2;
static const unsigned EP_RECV       = 7;
static const unsigned EP_MEM        = 8;
static const unsigned EP_DATA       = 9;
static const unsigned CAP_RBUF      = 2;
// the time for one 64 block; determined by ALADDIN and picking the sweet spot
// between area, power and performance. based on the SHA256 algorithm from:
// https://github.com/B-Con/crypto-algorithms/blob/master/sha256.c
static const Cycles BLOCK_TIME      = Cycles(85);
static const size_t MSG_SIZE        = 64;
static const Addr MSG_ADDR          = 0x2000;
static const Addr BUF_ADDR          = 0x4000;

static const Addr RCTMUX_FLAGS      = 0x2ff8;

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
    "REPLY_SYSCALL",
    "REPLY_FETCH",
    "REPLY_READ_ADDR",
    "REPLY_ACK",

    "CTX_SAVE",
    "CTX_SAVE_WRITE",
    "CTX_SAVE_SEND",
    "CTX_SAVE_WAIT",
    "CTX_SAVE_DONE",
    "CTX_WAIT",

    "CTX_CHECK",
    "CTX_RESTORE",
    "CTX_RESTORE_WAIT",
    "CTX_RESTORE_READ",
    "CTX_RESTORE_DONE",
};

Addr
DtuAccelHash::getRegAddr(DtuReg reg)
{
    return static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

Addr
DtuAccelHash::getRegAddr(CmdReg reg)
{
    Addr result = sizeof(RegFile::reg_t) * numDtuRegs;

    result += static_cast<Addr>(reg) * sizeof(RegFile::reg_t);

    return result;
}

Addr
DtuAccelHash::getRegAddr(unsigned reg, unsigned epid)
{
    Addr result = sizeof(RegFile::reg_t) * (numDtuRegs + numCmdRegs);

    result += epid * numEpRegs * sizeof(RegFile::reg_t);

    result += reg * sizeof(RegFile::reg_t);

    return result;
}

bool
DtuAccelHash::CpuPort::recvTimingResp(PacketPtr pkt)
{
    dtutest.completeRequest(pkt);
    return true;
}

void
DtuAccelHash::CpuPort::recvReqRetry()
{
    dtutest.recvRetry();
}

DtuAccelHash::DtuAccelHash(const DtuAccelHashParams *p)
  : MemObject(p),
    system(p->system),
    tickEvent(this),
    port("port", this),
    bufSize(p->buf_size),
    maxDataSize(p->max_data_size),
    chunkSize(system->cacheLineSize()),
    irqPending(false),
    state(State::IDLE),
    hash(),
    ctxOffset(),
    msgAddr(),
    masterId(system->getMasterId(name())),
    id(p->id),
    atomic(system->isAtomicMode()),
    reg_base(p->regfile_base_addr),
    retryPkt(nullptr),
    connector()
{
    static_assert(sizeof(DtuAccelHashAlgorithm) % 64 == 0, "Hash state size invalid");

    DTUMemory *sys = dynamic_cast<DTUMemory*>(system);
    haveVM = !sys->hasMem(id);
    // if we don't have VM, we have an SPM, which supports larger chunks
    if (!haveVM)
        chunkSize = maxDataSize;

    // kick things into action
    schedule(tickEvent, curTick());
}

BaseMasterPort &
DtuAccelHash::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return MemObject::getMasterPort(if_name, idx);
}

bool
DtuAccelHash::sendPkt(PacketPtr pkt)
{
    DPRINTF(DtuAccelAccess, "Send %s %s request at address 0x%x\n",
        atomic ? "atomic" : "timed",
        pkt->isWrite() ? "write" : "read",
        pkt->getAddr());

    if (atomic)
    {
        port.sendAtomic(pkt);
        completeRequest(pkt);
    }
    else if (!port.sendTimingReq(pkt))
    {
        retryPkt = pkt;
        return false;
    }

    return true;
}

void
DtuAccelHash::recvRetry()
{
    assert(retryPkt);
    if (port.sendTimingReq(retryPkt))
    {
        DPRINTF(DtuAccelAccess, "Proceeding after successful retry\n");

        retryPkt = nullptr;
    }
}

PacketPtr
DtuAccelHash::createPacket(Addr paddr,
                           size_t size,
                           MemCmd cmd = MemCmd::WriteReq)
{
    Request::Flags flags;

    auto req = new Request(paddr, size, flags, masterId);
    req->setContext(id);

    auto pkt = new Packet(req, cmd);
    auto pkt_data = new uint8_t[size];
    pkt->dataDynamic(pkt_data);

    return pkt;
}

PacketPtr
DtuAccelHash::createDtuRegPkt(Addr reg,
                              RegFile::reg_t value,
                              MemCmd cmd = MemCmd::WriteReq)
{
    auto pkt = createPacket(reg_base + reg, sizeof(RegFile::reg_t), cmd);
    *pkt->getPtr<RegFile::reg_t>() = value;
    return pkt;
}

PacketPtr
DtuAccelHash::createDtuCmdPkt(uint64_t cmd,
                              uint64_t data,
                              uint64_t size,
                              uint64_t off)
{
    static_assert(static_cast<int>(CmdReg::COMMAND) == 0, "");
    static_assert(static_cast<int>(CmdReg::ABORT) == 1, "");
    static_assert(static_cast<int>(CmdReg::DATA_ADDR) == 2, "");
    static_assert(static_cast<int>(CmdReg::DATA_SIZE) == 3, "");
    static_assert(static_cast<int>(CmdReg::OFFSET) == 4, "");
    static_assert(static_cast<int>(CmdReg::REPLY_EPID) == 5, "");

    auto pkt = createPacket(reg_base + getRegAddr(CmdReg::COMMAND),
                            sizeof(RegFile::reg_t) * 6,
                            MemCmd::WriteReq);

    RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
    regs[0] = cmd;
    regs[1] = 0;
    regs[2] = data;
    regs[3] = size;
    regs[4] = off;
    regs[5] = EP_SYSR;
    return pkt;
}

void
DtuAccelHash::freePacket(PacketPtr pkt)
{
    delete pkt->req;
    // the packet will delete the data
    delete pkt;
}

size_t DtuAccelHash::getStateSize() const
{
    // if we fill the buffer, we do not need to save it, since we don't
    // interrupt that operation.
    if (hash.isAutonomous())
        return sizeof(hash);
    return sizeof(hash) + bufSize;
}

void
DtuAccelHash::completeRequest(PacketPtr pkt)
{
    Request* req = pkt->req;

    DPRINTF(DtuAccelState, "[%s] Got response from memory\n",
        stateNames[static_cast<size_t>(state)]);

    DPRINTF(DtuAccelAccess, "Completing %s at address %x:%lu %s\n",
        pkt->isWrite() ? "write" : "read",
        req->getPaddr(),
        req->getSize(),
        pkt->isError() ? "error" : "success");

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
            case State::CTX_WAIT:
            {
                assert(false);
                break;
            }

            case State::CTX_SAVE:
            {
                ctxOffset = 0;
                state = State::CTX_SAVE_WRITE;
                break;
            }
            case State::CTX_SAVE_WRITE:
            {
                ctxOffset += pkt->getSize();
                if (ctxOffset == sizeof(hash))
                {
                    ctxOffset = 0;
                    if (haveVM)
                        state = State::CTX_SAVE_DONE;
                    else
                        state = State::CTX_SAVE_SEND;
                }
                break;
            }
            case State::CTX_SAVE_SEND:
            {
                state = State::CTX_SAVE_WAIT;
                break;
            }
            case State::CTX_SAVE_WAIT:
            {
                RegFile::reg_t reg =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if ((reg & 0xF) == 0)
                {
                    // don't continue on errors here; maybe we don't have the
                    // memory EP yet.
                    if ((reg >> 13) != 0 || ctxOffset == getStateSize())
                        state = State::CTX_SAVE_DONE;
                    else
                        state = State::CTX_SAVE_SEND;
                }
                break;
            }
            case State::CTX_SAVE_DONE:
            {
                state = State::CTX_WAIT;
                break;
            }

            case State::CTX_CHECK:
            {
                uint64_t val = *pkt->getConstPtr<uint64_t>();
                if (val & RCTMuxCtrl::RESTORE)
                {
                    ctxOffset = 0;
                    if (haveVM)
                        state = State::CTX_RESTORE_READ;
                    else
                        state = State::CTX_RESTORE;
                }
                else if((val & RCTMuxCtrl::WAITING) && (~val & RCTMuxCtrl::STORE))
                    state = State::CTX_RESTORE_DONE;
                else
                    state = State::FETCH_MSG;
                break;
            }
            case State::CTX_RESTORE:
            {
                state = State::CTX_RESTORE_WAIT;
                break;
            }
            case State::CTX_RESTORE_WAIT:
            {
                RegFile::reg_t reg =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if ((reg & 0xF) == 0)
                {
                    if (ctxOffset == getStateSize())
                    {
                        ctxOffset = 0;
                        state = State::CTX_RESTORE_READ;
                    }
                    else
                        state = State::CTX_RESTORE;
                }
                break;
            }
            case State::CTX_RESTORE_READ:
            {
                memcpy((char*)&hash + ctxOffset,
                       pkt->getPtr<char>(),
                       pkt->getSize());

                ctxOffset += pkt->getSize();
                if (ctxOffset == sizeof(hash))
                    state = State::CTX_RESTORE_DONE;
                break;
            }
            case State::CTX_RESTORE_DONE:
            {
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
                    DPRINTF(DtuAccel, "Received message @ %p\n", msgAddr);
                    state = State::READ_MSG;
                }
                else
                    state = State::IDLE;
                break;
            }
            case State::READ_MSG:
            {
                const uint64_t *args =
                    reinterpret_cast<const uint64_t*>(
                        pkt_data + sizeof(Dtu::MessageHeader));

                DPRINTF(DtuAccel, "  cmd=%lld arg1=%#llx arg2=%#llx\n",
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
                        memOff = args[1];
                        dataSize = args[2];
                        dataOff = 0;
                        if (hash.isAutonomous())
                            state = State::READ_DATA;
                        else
                        {
                            dataOff = lastSize = dataSize;
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
                        DPRINTF(DtuAccel, "Hash: %s\n", ss.str().c_str());

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
                RegFile::reg_t reg =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if ((reg & 0xF) == 0)
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
                    delay = Cycles(BLOCK_TIME * (lastSize / 64));
                    DPRINTF(DtuAccel, "Hash generation took %llu cycles\n", delay);

                    // decrease it by the time we've already spent reading the
                    // data from SPM, because that's already included in the
                    // BLOCK_TIME.
                    // TODO if we use caches, this is not correct
                    if (delay > (curCycle() - hashStart))
                        delay = delay - (curCycle() - hashStart);
                    else
                        delay = Cycles(1);

                    if (dataOff == dataSize)
                        state = State::STORE_REPLY;
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
                RegFile::reg_t reg =
                    *reinterpret_cast<const RegFile::reg_t*>(pkt_data);
                if ((reg & 0xF) == 0)
                {
                    if ((reg >> 13) == 0)
                        state = State::CTX_CHECK;
                    else
                        state = State::REPLY_ERROR;
                }
                break;
            }
            case State::REPLY_ERROR:
            {
                state = State::REPLY_SYSCALL;
                break;
            }
            case State::REPLY_SYSCALL:
            {
                state = State::REPLY_FETCH;
                break;
            }
            case State::REPLY_FETCH:
            {
                state = State::REPLY_READ_ADDR;
                break;
            }
            case State::REPLY_READ_ADDR:
            {
                const RegFile::reg_t *regs = pkt->getConstPtr<RegFile::reg_t>();
                if(regs[0])
                {
                    sysreplyAddr = regs[0];
                    state = State::REPLY_ACK;
                }
                else
                    state = State::REPLY_FETCH;
                break;
            }
            case State::REPLY_ACK:
            {
                state = State::CTX_CHECK;
                break;
            }
        }
    }

    freePacket(pkt);

    // kick things into action again
    schedule(tickEvent, clockEdge(delay));
}

void
DtuAccelHash::interrupt()
{
    if (state == State::IDLE)
    {
        state = State::CTX_SAVE;
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
    else
        irqPending = true;
}

void
DtuAccelHash::wakeup()
{
    if (state == State::IDLE || state == State::CTX_WAIT)
    {
        state = State::CTX_CHECK;
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void
DtuAccelHash::tick()
{
    PacketPtr pkt = nullptr;

    DPRINTF(DtuAccelState, "[%s] tick\n",
        stateNames[static_cast<size_t>(state)]);

    switch(state)
    {
        case State::IDLE:
        case State::CTX_WAIT:
        {
            if (connector)
                DPRINTFS(DtuConnector, connector, "Suspending accelerator\n");
            break;
        }

        case State::CTX_SAVE:
        {
            Addr regAddr = getRegAddr(CmdReg::ABORT);
            uint64_t value = Dtu::Command::ABORT_VPE;
            pkt = createDtuRegPkt(regAddr, value, MemCmd::WriteReq);
            break;
        }
        case State::CTX_SAVE_WRITE:
        {
            size_t rem = sizeof(hash) - ctxOffset;
            size_t size = std::min(chunkSize, rem);
            pkt = createPacket((BUF_ADDR - sizeof(hash)) + ctxOffset,
                               size,
                               MemCmd::WriteReq);
            memcpy(pkt->getPtr<uint8_t>(), (char*)&hash + ctxOffset, size);
            break;
        }
        case State::CTX_SAVE_SEND:
        {
            size_t rem = getStateSize() - ctxOffset;
            size_t size = std::min(maxDataSize, rem);
            pkt = createDtuCmdPkt(Dtu::Command::WRITE | (EP_MEM << 4),
                                  (BUF_ADDR - sizeof(hash)) + ctxOffset,
                                  size,
                                  ctxOffset);
            ctxOffset += size;
            break;
        }
        case State::CTX_SAVE_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
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
            size_t rem = getStateSize() - ctxOffset;
            size_t size = std::min(maxDataSize, rem);
            pkt = createDtuCmdPkt(Dtu::Command::READ | (EP_MEM << 4),
                                  (BUF_ADDR - sizeof(hash)) + ctxOffset,
                                  size,
                                  ctxOffset);
            ctxOffset += size;
            break;
        }
        case State::CTX_RESTORE_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::CTX_RESTORE_READ:
        {
            size_t rem = sizeof(hash) - ctxOffset;
            size_t size = std::min(chunkSize, rem);
            pkt = createPacket((BUF_ADDR - sizeof(hash)) + ctxOffset,
                               size,
                               MemCmd::ReadReq);
            break;
        }
        case State::CTX_RESTORE_DONE:
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
            lastSize = std::min(maxDataSize, dataSize - dataOff);
            pkt = createDtuCmdPkt(Dtu::Command::READ | (EP_DATA << 4),
                                  BUF_ADDR,
                                  lastSize,
                                  memOff + dataOff);
            dataOff += lastSize;
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
            pkt = createDtuCmdPkt(Dtu::Command::REPLY | (EP_RECV << 4),
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
            reply.sys.opcode = 15;
            reply.sys.cap = CAP_RBUF;
            reply.sys.msgaddr = msgAddr;
            reply.sys.event = 0;
            reply.sys.len = replySize;
            reply.sys.event = 0;

            pkt = createPacket(MSG_ADDR, sizeof(reply), MemCmd::WriteReq);
            memcpy(pkt->getPtr<void>(), &reply, sizeof(reply));
            break;
        }
        case State::REPLY_SYSCALL:
        {
            pkt = createDtuCmdPkt(Dtu::Command::SEND | (EP_SYSS << 4),
                                  MSG_ADDR,
                                  sizeof(reply),
                                  0);
            break;
        }
        case State::REPLY_FETCH:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            uint64_t value = Dtu::Command::FETCH_MSG | (EP_SYSR << 4);
            pkt = createDtuRegPkt(regAddr, value, MemCmd::WriteReq);
            break;
        }
        case State::REPLY_READ_ADDR:
        {
            Addr regAddr = getRegAddr(CmdReg::OFFSET);
            pkt = createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::REPLY_ACK:
        {
            pkt = createDtuCmdPkt(Dtu::Command::ACK_MSG | (EP_SYSR << 4),
                                  0,
                                  0,
                                  sysreplyAddr);
            break;
        }
    }

    if (pkt != nullptr)
        sendPkt(pkt);
}

DtuAccelHash*
DtuAccelHashParams::create()
{
    return new DtuAccelHash(this);
}
