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

#include "cpu/testers/tcuaborttest/tcuaborttest.hh"
#include "debug/TcuAbortTest.hh"
#include "debug/TcuAbortTestAccess.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/regfile.hh"
#include "sim/pe_memory.hh"
#include "sim/sim_exit.hh"

static const Addr DATA_ADDR         = 0x1000;
static const Addr DEST_ADDR         = 0x2000;
static const Addr RECV_ADDR         = 0x3000;
static const unsigned EP_MEM        = 0;
static const unsigned EP_SEND       = 1;
static const unsigned EP_RECV       = 2;
static const unsigned EP_REPLY      = 3;
static const unsigned TEST_COUNT    = 4;

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
    "WAIT_ABORT",
    "WAIT_CMD",
};

static const char *testNames[] =
{
    "WRITE",
    "READ",
    "SEND",
    "REPLY",
};

static const char *abortNames[] =
{
    "NONE",
    "LOCAL",
    "REMOTE",
};

Addr
TcuAbortTest::getRegAddr(TcuReg reg)
{
    return static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

Addr
TcuAbortTest::getRegAddr(PrivReg reg)
{
    return TcuTlb::PAGE_SIZE * 2 +
           static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

Addr
TcuAbortTest::getRegAddr(CmdReg reg)
{
    Addr result = sizeof(RegFile::reg_t) * numTcuRegs;

    result += static_cast<Addr>(reg) * sizeof(RegFile::reg_t);

    return result;
}

Addr
TcuAbortTest::getRegAddr(unsigned reg, unsigned epid)
{
    Addr result = sizeof(RegFile::reg_t) * (numTcuRegs + numCmdRegs);

    result += epid * numEpRegs * sizeof(RegFile::reg_t);

    result += reg * sizeof(RegFile::reg_t);

    return result;
}

bool
TcuAbortTest::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tcutest.completeRequest(pkt);
    return true;
}

void
TcuAbortTest::CpuPort::recvReqRetry()
{
    tcutest.recvRetry();
}

TcuAbortTest::TcuAbortTest(const TcuAbortTestParams *p)
  : ClockedObject(p),
    tickEvent(this),
    port("port", this),
    state(State::INIT_MEM),
    substate(SubState::START),
    testNo(0),
    delay(0),
    abortType(),
    abortTypes(),
    abortStart(0),
    system(p->system),
    // TODO parameterize
    reg_base(0xF0000000),
    masterId(p->system->getMasterId(this, name())),
    atomic(p->system->isAtomicMode()),
    retryPkt(nullptr)
{
    id = p->id;

    // kick things into action
    schedule(tickEvent, curTick());
}

Port &
TcuAbortTest::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return SimObject::getPort(if_name, idx);
}

bool
TcuAbortTest::sendPkt(PacketPtr pkt)
{
    DPRINTF(TcuAbortTestAccess, "Send %s %s request at address 0x%x\n",
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
TcuAbortTest::recvRetry()
{
    assert(retryPkt);
    if (port.sendTimingReq(retryPkt))
    {
        DPRINTF(TcuAbortTestAccess, "Proceeding after successful retry\n");

        retryPkt = nullptr;
    }
}

PacketPtr
TcuAbortTest::createPacket(Addr paddr,
                          size_t size,
                          MemCmd cmd = MemCmd::WriteReq)
{
    Request::Flags flags;

    auto req = std::make_shared<Request>(paddr, size, flags, masterId);
    req->setContext(id);

    auto pkt = new Packet(req, cmd);
    auto pkt_data = new uint8_t[size];
    pkt->dataDynamic(pkt_data);

    return pkt;
}

PacketPtr
TcuAbortTest::createTcuRegisterPkt(Addr reg,
                                  RegFile::reg_t value,
                                  MemCmd cmd = MemCmd::WriteReq)
{
    auto pkt = createPacket(reg_base + reg, sizeof(RegFile::reg_t), cmd);
    *pkt->getPtr<RegFile::reg_t>() = value;
    return pkt;
}

PacketPtr
TcuAbortTest::createCommandPkt(Tcu::Command::Opcode cmd,
                               unsigned ep,
                               Addr data,
                               Addr size,
                               Addr arg0,
                               Addr arg1)
{
    static_assert(static_cast<int>(CmdReg::COMMAND) == 0, "");
    static_assert(static_cast<int>(CmdReg::DATA) == 1, "");
    static_assert(static_cast<int>(CmdReg::ARG1) == 2, "");

    auto pkt = createPacket(reg_base + getRegAddr(CmdReg::COMMAND),
                            sizeof(RegFile::reg_t) * 3,
                            MemCmd::WriteReq);

    Tcu::Command::Bits cmdreg = 0;
    cmdreg.opcode = static_cast<RegFile::reg_t>(cmd);
    cmdreg.epid = ep;
    cmdreg.arg = arg0;

    RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
    regs[0] = cmdreg;
    regs[1] = DataReg(data, size).value();
    regs[2] = arg1;
    return pkt;
}

void
TcuAbortTest::completeRequest(PacketPtr pkt)
{
    RequestPtr req = pkt->req;

    DPRINTF(TcuAbortTest, "[%s:%d,%s:%d] Got response from memory\n",
        stateNames[static_cast<size_t>(state)],
        testNo,
        substateNames[static_cast<size_t>(substate)],
        delay);

    DPRINTF(TcuAbortTestAccess, "Completing %s at address %x %s\n",
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
                        substate = SubState::WAIT_ABORT;
                        break;
                    case SubState::WAIT_ABORT:
                    {
                        RegFile::reg_t reg = *pkt->getPtr<RegFile::reg_t>();
                        if ((reg & 0xF) == 0)
                        {
                            abortType = reg >> 4;
                            DPRINTF(TcuAbortTest,
                                    "Command aborted as %#x\n", abortType);
                            abortTypes |= 1 << abortType;
                            substate = SubState::WAIT_CMD;
                        }
                        break;
                    }
                    case SubState::WAIT_CMD:
                    {
                        RegFile::reg_t reg = *pkt->getPtr<RegFile::reg_t>();
                        if ((reg & 0xF) == 0)
                        {
                            if (((reg >> 20) & 0xF) == 0)
                            {
                                auto all = 1 << uint(Tcu::AbortType::NONE) |
                                           1 << uint(Tcu::AbortType::LOCAL);
                                // messages are not remote aborted
                                if (testNo < 2)
                                    all |= 1 << uint(Tcu::AbortType::REMOTE);
                                if (abortTypes != all)
                                {
                                    inform("! %s:%d  saw aborts %#x"
                                           ", expected %#x FAILED\n",
                                           __FUNCTION__, __LINE__,
                                           abortTypes, all);
                                }

                                abortTypes = 0;

                                if (++testNo == TEST_COUNT)
                                    state = State::STOP;
                                else
                                {
                                    delay = 0;
                                    substate = SubState::START;
                                }
                            }
                            else
                            {
                                delay++;
                                substate = SubState::START;
                            }
                        }
                        break;
                    }
                }
                break;
        }
    }

    // the packet will delete the data
    delete pkt;

    // kick things into action again
    schedule(tickEvent, clockEdge(schedDelay));
}

void
TcuAbortTest::tick()
{
    PacketPtr pkt = nullptr;

    DPRINTF(TcuAbortTest, "[%s:%d,%s:%d] tick\n",
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
            state = State::INIT_MEM_EP;
            break;
        }

        case State::INIT_MEM_EP:
        {
            pkt = createPacket(reg_base + getRegAddr(0, EP_MEM),
                               sizeof(RegFile::reg_t) * numEpRegs,
                               MemCmd::WriteReq);

            RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
            regs[0] = static_cast<RegFile::reg_t>(EpType::MEMORY) |
                      // our VPE
                      static_cast<RegFile::reg_t>(Tcu::INVALID_VPE_ID) << 3 |
                      // permissions
                      static_cast<RegFile::reg_t>(0x3) << 19 |
                      // target PE
                      static_cast<RegFile::reg_t>(id) << 23 |
                      // target VPE
                      static_cast<RegFile::reg_t>(Tcu::INVALID_VPE_ID) << 31;
                      // base address
            regs[1] = 0x0;
                      // size
            regs[2] = 0xFFFFFFFFFFFFFFFF;
            break;
        }

        case State::INIT_SEND_EP:
        {
            pkt = createPacket(reg_base + getRegAddr(0, EP_SEND),
                               sizeof(RegFile::reg_t) * numEpRegs,
                               MemCmd::WriteReq);

            RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
            regs[0] = static_cast<RegFile::reg_t>(EpType::SEND) |
                      // our VPE
                      static_cast<RegFile::reg_t>(Tcu::INVALID_VPE_ID) << 3 |
                      // cur credits
                      static_cast<RegFile::reg_t>(Tcu::CREDITS_UNLIM) << 19 |
                      // max credits
                      static_cast<RegFile::reg_t>(Tcu::CREDITS_UNLIM) << 25 |
                      // message order
                      static_cast<RegFile::reg_t>(10) << 31;
                      // target PE
            regs[1] = (static_cast<RegFile::reg_t>(id) << 16) |
                      // target EP
                      (static_cast<RegFile::reg_t>(EP_RECV) << 0);
                      // label
            regs[2] = 0;
            break;
        }

        case State::INIT_RECV_EP:
        {
            pkt = createPacket(reg_base + getRegAddr(0, EP_RECV),
                               sizeof(RegFile::reg_t) * numEpRegs,
                               MemCmd::WriteReq);

            RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
            regs[0] = static_cast<RegFile::reg_t>(EpType::RECEIVE) |
                      // our VPE
                      static_cast<RegFile::reg_t>(Tcu::INVALID_VPE_ID) << 3 |
                      // reply EPs
                      static_cast<RegFile::reg_t>(EP_REPLY) << 19 |
                      // buf size
                      static_cast<RegFile::reg_t>(10) << 35 |
                      // msg size
                      static_cast<RegFile::reg_t>(10) << 41;
                      // buf addr
            regs[1] = RECV_ADDR;
                      // occupied + unread
            regs[2] = 0;
            break;
        }

        case State::TESTS:
        {
            switch (substate)
            {
                case SubState::START:
                {
                    if (delay == 0)
                    {
                        inform("Testing \"abort %s\" in %s:%d\n",
                            testNames[testNo], __FUNCTION__, __LINE__);
                    }

                    if (testNo == 0)
                    {
                        pkt = createCommandPkt(Tcu::Command::WRITE,
                                               EP_MEM,
                                               DATA_ADDR,
                                               system->cacheLineSize() * 8,
                                               DEST_ADDR);
                    }
                    else if (testNo == 1)
                    {
                        pkt = createCommandPkt(Tcu::Command::READ,
                                               EP_MEM,
                                               DATA_ADDR,
                                               system->cacheLineSize() * 8,
                                               DEST_ADDR);
                    }
                    else if (testNo == 2)
                    {
                        pkt = createCommandPkt(Tcu::Command::SEND,
                                               EP_SEND,
                                               DATA_ADDR,
                                               system->cacheLineSize() * 8,
                                               EP_RECV,
                                               0);
                    }
                    else if (testNo == 3)
                    {
                        pkt = createCommandPkt(Tcu::Command::REPLY,
                                               EP_RECV,
                                               DATA_ADDR,
                                               system->cacheLineSize() * 8,
                                               0);
                    }
                    break;
                }

                case SubState::ABORT:
                {
                    Addr regAddr = getRegAddr(PrivReg::PRIV_CMD);
                    pkt = createTcuRegisterPkt(regAddr,
                                               Tcu::PrivCommand::ABORT_CMD,
                                               MemCmd::WriteReq);
                    abortStart = curTick();
                    break;
                }

                case SubState::WAIT_ABORT:
                {
                    Addr regAddr = getRegAddr(PrivReg::PRIV_CMD);
                    pkt = createTcuRegisterPkt(regAddr, 0, MemCmd::ReadReq);
                    break;
                }

                case SubState::WAIT_CMD:
                {
                    inform("  Abort %u of %s took %lu ticks (%s)\n",
                        delay, testNames[testNo], curTick() - abortStart,
                        abortNames[abortType]);

                    Addr regAddr = getRegAddr(CmdReg::COMMAND);
                    pkt = createTcuRegisterPkt(regAddr, 0, MemCmd::ReadReq);
                    break;
                }
            }
            break;
        }

        case State::STOP:
        {
            Tick when = curTick() + SimClock::Int::ns;
            inform("[kernel  @0] Shutting down\n");
            exitSimLoop("All tests done", 0, when, 0, false);
            return;
        }
    }

    // Schedule next tick if the packet was successfully send.
    // Otherwise block until a retry is received.

    if (pkt == nullptr)
        schedule(tickEvent, clockEdge(Cycles(1)));
    else
        sendPkt(pkt);
}

TcuAbortTest*
TcuAbortTestParams::create()
{
    return new TcuAbortTest(this);
}
