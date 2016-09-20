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

#include "cpu/testers/dtuaborttest/dtuaborttest.hh"
#include "debug/DtuAbortTest.hh"
#include "debug/DtuAbortTestAccess.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/regfile.hh"
#include "sim/dtu_memory.hh"
#include "base/misc.hh"

// start at 0x2000, because it is 1:1 mapped and we use the first 2 for PTs
static const Addr TEMP_ADDR         = DtuTlb::PAGE_SIZE * 2;
static const Addr DATA_ADDR         = DtuTlb::PAGE_SIZE * 3;
static const Addr DEST_ADDR         = DtuTlb::PAGE_SIZE * 4;
static const Addr RECV_ADDR         = DtuTlb::PAGE_SIZE * 5;
static const unsigned EP_MEM        = 0;
static const unsigned EP_SEND       = 1;
static const unsigned EP_RECV       = 2;
static const unsigned TEST_COUNT    = 4;
static const Tick MAX_ABORT_TIME    = 30000;

static const char *stateNames[] =
{
    "IDLE",
    "INIT_MEM",
    "INIT_MEM_EP",
    "INIT_SEND_EP",
    "INIT_RECV_EP",
    "TESTS",
    "STOP",
};

static const char *substateNames[] =
{
    "START",
    "ABORT",
    "WAIT",
    "INVLPG_DATA",
    "INVLPG_CMD",
    "INVLPG_WAIT",
};

Addr
DtuAbortTest::getRegAddr(DtuReg reg)
{
    return static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

Addr
DtuAbortTest::getRegAddr(CmdReg reg)
{
    Addr result = sizeof(RegFile::reg_t) * numDtuRegs;

    result += static_cast<Addr>(reg) * sizeof(RegFile::reg_t);

    return result;
}

Addr
DtuAbortTest::getRegAddr(unsigned reg, unsigned epid)
{
    Addr result = sizeof(RegFile::reg_t) * (numDtuRegs + numCmdRegs);

    result += epid * numEpRegs * sizeof(RegFile::reg_t);

    result += reg * sizeof(RegFile::reg_t);

    return result;
}

bool
DtuAbortTest::CpuPort::recvTimingResp(PacketPtr pkt)
{
    dtutest.completeRequest(pkt);
    return true;
}

void
DtuAbortTest::CpuPort::recvReqRetry()
{
    dtutest.recvRetry();
}

DtuAbortTest::DtuAbortTest(const DtuAbortTestParams *p)
  : MemObject(p),
    tickEvent(this),
    port("port", this),
    state(State::INIT_MEM),
    substate(SubState::START),
    testNo(0),
    delay(1),
    abortStart(0),
    system(p->system),
    // TODO parameterize
    reg_base(0xF0000000),
    masterId(p->system->getMasterId(name())),
    atomic(p->system->isAtomicMode()),
    retryPkt(nullptr)
{
    id = p->id;

    // kick things into action
    schedule(tickEvent, curTick());
}

BaseMasterPort &
DtuAbortTest::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return MemObject::getMasterPort(if_name, idx);
}

bool
DtuAbortTest::sendPkt(PacketPtr pkt)
{
    DPRINTF(DtuAbortTestAccess, "Send %s %s request at address 0x%x\n",
                           atomic ? "atomic" : "timed",
                           pkt->isWrite() ? "write" : "read",
                           pkt->getAddr());

    if (atomic)
    {
        port.sendAtomic(pkt);
        completeRequest(pkt);
    }
    else
    {
        bool retry = !port.sendTimingReq(pkt);

        if (retry)
        {
            retryPkt = pkt;
            return false;
        }
    }

    return true;
}

void
DtuAbortTest::recvRetry()
{
    assert(retryPkt);
    if (port.sendTimingReq(retryPkt))
    {
        DPRINTF(DtuAbortTestAccess, "Proceeding after successful retry\n");

        retryPkt = nullptr;
    }
}

PacketPtr
DtuAbortTest::createPacket(Addr paddr,
                          size_t size,
                          MemCmd cmd = MemCmd::WriteReq)
{
    Request::Flags flags;

    auto req = new Request(paddr, size, flags, masterId);
    req->setThreadContext(id, 0);

    auto pkt = new Packet(req, cmd);
    auto pkt_data = new uint8_t[size];
    pkt->dataDynamic(pkt_data);

    return pkt;
}

PacketPtr
DtuAbortTest::createDtuRegisterPkt(Addr reg,
                                  RegFile::reg_t value,
                                  MemCmd cmd = MemCmd::WriteReq)
{
    auto pkt = createPacket(reg_base + reg, sizeof(RegFile::reg_t), cmd);
    *pkt->getPtr<RegFile::reg_t>() = value;
    return pkt;
}

PacketPtr
DtuAbortTest::createCommandPkt(Dtu::Command::Opcode cmd,
                               unsigned ep,
                               Addr data,
                               Addr size,
                               Addr off,
                               unsigned reply_ep = 0)
{
    static_assert(static_cast<int>(CmdReg::COMMAND) == 0, "");
    static_assert(static_cast<int>(CmdReg::ABORT) == 1, "");
    static_assert(static_cast<int>(CmdReg::DATA_ADDR) == 2, "");
    static_assert(static_cast<int>(CmdReg::DATA_SIZE) == 3, "");
    static_assert(static_cast<int>(CmdReg::OFFSET) == 4, "");
    static_assert(static_cast<int>(CmdReg::REPLY_EPID) == 5, "");

    PacketPtr pkt = createPacket(reg_base + getRegAddr(CmdReg::COMMAND),
                                 sizeof(RegFile::reg_t) * 6,
                                 MemCmd::WriteReq);

    RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
    regs[0] = static_cast<RegFile::reg_t>(cmd) | (ep << 4);
    regs[1] = 0;
    regs[2] = data;
    regs[3] = size;
    regs[4] = off;
    regs[5] = reply_ep;
    return pkt;
}

void
DtuAbortTest::completeRequest(PacketPtr pkt)
{
    Request* req = pkt->req;

    DPRINTF(DtuAbortTest, "[%s:%d,%s:%d] Got response from memory\n",
        stateNames[static_cast<size_t>(state)],
        testNo,
        substateNames[static_cast<size_t>(substate)],
        delay);

    DPRINTF(DtuAbortTestAccess, "Completing %s at address %x %s\n",
                           pkt->isWrite() ? "write" : "read",
                           req->getPaddr(),
                           pkt->isError() ? "error" : "success");

    Cycles schedDelay(1);

    if (pkt->isError())
    {
        warn("%s access failed at %#x\n",
             pkt->isWrite() ? "Write" : "Read", req->getPaddr());
    }
    else
    {
        switch (state)
        {
            case State::IDLE:
            case State::INIT_MEM:
            case State::STOP:
                assert(false);
                break;

            case State::INIT_MEM_EP:
                state = State::INIT_SEND_EP;
                break;
            case State::INIT_SEND_EP:
                state = State::INIT_RECV_EP;
                break;
            case State::INIT_RECV_EP:
                state = State::TESTS;
                break;

            case State::TESTS:
                switch (substate)
                {
                    case SubState::START:
                        substate = SubState::ABORT;
                        schedDelay = Cycles(delay);
                        break;
                    case SubState::ABORT:
                        substate = SubState::WAIT;
                        break;
                    case SubState::WAIT:
                    {
                        RegFile::reg_t reg = *pkt->getPtr<RegFile::reg_t>();
                        if ((reg & 0xF) == 0)
                        {
                            if ((reg >> 13) == 0)
                            {
                                if (++testNo == TEST_COUNT)
                                    state = State::STOP;
                                else
                                {
                                    delay = 0;
                                    substate = SubState::INVLPG_DATA;
                                }
                            }
                            else
                                substate = SubState::INVLPG_DATA;
                        }
                        break;
                    }

                    case SubState::INVLPG_DATA:
                        substate = SubState::INVLPG_CMD;
                        break;
                    case SubState::INVLPG_CMD:
                        substate = SubState::INVLPG_WAIT;
                        break;
                    case SubState::INVLPG_WAIT:
                    {
                        RegFile::reg_t reg = *pkt->getPtr<RegFile::reg_t>();
                        if ((reg & 0xF) == 0)
                        {
                            substate = SubState::START;
                            delay++;
                        }
                        break;
                    }
                }
                break;
        }
    }

    delete pkt->req;

    // the packet will delete the data
    delete pkt;

    // kick things into action again
    schedule(tickEvent, clockEdge(schedDelay));
}

void
DtuAbortTest::tick()
{
    PacketPtr pkt = nullptr;

    DPRINTF(DtuAbortTest, "[%s:%d,%s:%d] tick\n",
        stateNames[static_cast<size_t>(state)],
        testNo,
        substateNames[static_cast<size_t>(substate)],
        delay);

    switch (state)
    {
        case State::IDLE:
            break;

        case State::INIT_MEM:
        {
            DTUMemory *sys = dynamic_cast<DTUMemory*>(system);
            if (sys)
            {
                sys->mapSegment(0, DtuTlb::PAGE_SIZE * 6, DtuTlb::IRWX);
            }
            state = State::INIT_MEM_EP;
            break;
        }

        case State::INIT_MEM_EP:
        {
            pkt = createPacket(reg_base + getRegAddr(0, EP_MEM),
                               sizeof(RegFile::reg_t) * numEpRegs,
                               MemCmd::WriteReq);

            RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
            regs[0] = (static_cast<RegFile::reg_t>(EpType::MEMORY) << 61) |
                      0xFFFFFFFF;                   // size
            regs[1] = 0x0;                          // base address
            regs[2] = (Dtu::INVALID_VPE_ID << 12) | // vpe id
                      (id << 4) |                   // core id
                      (0x7 << 0);                   // access
            break;
        }

        case State::INIT_SEND_EP:
        {
            pkt = createPacket(reg_base + getRegAddr(0, EP_SEND),
                               sizeof(RegFile::reg_t) * numEpRegs,
                               MemCmd::WriteReq);

            RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
            regs[0] = (static_cast<RegFile::reg_t>(EpType::SEND) << 61) |
                      (static_cast<RegFile::reg_t>(Dtu::INVALID_VPE_ID) << 16) |
                      (256 << 0);                   // max msg size
            regs[1] = (id << 24) |                  // target core
                      (EP_RECV << 16) |             // target EP
                      (256 << 0);                   // credits
            regs[2] = 0;                            // label
            break;
        }

        case State::INIT_RECV_EP:
        {
            pkt = createPacket(reg_base + getRegAddr(0, EP_RECV),
                               sizeof(RegFile::reg_t) * numEpRegs,
                               MemCmd::WriteReq);

            RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
            regs[0] = (static_cast<RegFile::reg_t>(EpType::RECEIVE) << 61) |
                      (static_cast<RegFile::reg_t>(256) << 32) | // max msg size
                      (4 << 16) |                   // size
                      (0 << 0);                     // msg count
            regs[1] = RECV_ADDR;                    // buf addr
            regs[2] = 0;                            // rd + wr offset
            break;
        }

        case State::TESTS:
        {
            switch (substate)
            {
                case SubState::START:
                {
                    if (testNo == 0)
                    {
                        pkt = createCommandPkt(Dtu::Command::WRITE,
                                               EP_MEM,
                                               DATA_ADDR,
                                               system->cacheLineSize() * 2,
                                               DEST_ADDR);
                    }
                    else if (testNo == 1)
                    {
                        pkt = createCommandPkt(Dtu::Command::READ,
                                               EP_MEM,
                                               DATA_ADDR,
                                               system->cacheLineSize() * 2,
                                               DEST_ADDR);
                    }
                    else if (testNo == 2)
                    {
                        pkt = createCommandPkt(Dtu::Command::SEND,
                                               EP_SEND,
                                               DATA_ADDR,
                                               system->cacheLineSize() * 2,
                                               0,
                                               EP_RECV);
                    }
                    else if (testNo == 3)
                    {
                        pkt = createCommandPkt(Dtu::Command::REPLY,
                                               EP_RECV,
                                               DATA_ADDR,
                                               system->cacheLineSize() * 2,
                                               RECV_ADDR);
                    }
                    break;
                }

                case SubState::ABORT:
                {
                    Addr regAddr = getRegAddr(CmdReg::ABORT);
                    pkt = createDtuRegisterPkt(regAddr,
                                               Dtu::Command::ABORT_CMD,
                                               MemCmd::WriteReq);
                    abortStart = curTick();
                    break;
                }

                case SubState::WAIT:
                {
                    Addr regAddr = getRegAddr(CmdReg::COMMAND);
                    pkt = createDtuRegisterPkt(regAddr, 0, MemCmd::ReadReq);
                    break;
                }

                case SubState::INVLPG_DATA:
                {
                    if (curTick() - abortStart > MAX_ABORT_TIME)
                    {
                        warn("Abort %u of test %u took %lu ticks\n",
                            delay, testNo, curTick() - abortStart);
                    }

                    pkt = createPacket(TEMP_ADDR,
                                       sizeof(RegFile::reg_t),
                                       MemCmd::WriteReq);

                    RegFile::reg_t cmd = 0;
                    cmd += static_cast<RegFile::reg_t>(Dtu::ExternCommand::INV_TLB);
                    *pkt->getPtr<RegFile::reg_t>() = cmd;
                    break;
                }

                case SubState::INVLPG_CMD:
                {
                    Addr off = reg_base + getRegAddr(DtuReg::EXT_CMD);
                    pkt = createCommandPkt(Dtu::Command::WRITE,
                                           EP_MEM,
                                           TEMP_ADDR,
                                           sizeof(RegFile::reg_t),
                                           off);
                    break;
                }

                case SubState::INVLPG_WAIT:
                {
                    Addr regAddr = getRegAddr(CmdReg::COMMAND);
                    pkt = createDtuRegisterPkt(regAddr, 0, MemCmd::ReadReq);
                    break;
                }
            }
            break;
        }

        case State::STOP:
        {
            exit_message("info", 1, "Test done");
            break;
        }
    }

    // Schedule next tick if the packet was successfully send.
    // Otherwise block until a retry is received.

    if (pkt == nullptr)
        schedule(tickEvent, clockEdge(Cycles(1)));
    else
        sendPkt(pkt);
}

DtuAbortTest*
DtuAbortTestParams::create()
{
    return new DtuAbortTest(this);
}
