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

#include "cpu/dtu-accel/accelerator.hh"
#include "debug/DtuConnector.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/regfile.hh"
#include "sim/dtu_memory.hh"

#include <iomanip>

const unsigned DtuAccel::EP_SYSS       = 0;
const unsigned DtuAccel::EP_SYSR       = 2;

const Addr DtuAccel::MSG_ADDR          = 0x2000;

const Addr DtuAccel::RCTMUX_YIELD      = 0x2ff0;
const Addr DtuAccel::RCTMUX_FLAGS      = 0x2ff8;

Addr
DtuAccel::getRegAddr(DtuReg reg)
{
    return static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

Addr
DtuAccel::getRegAddr(CmdReg reg)
{
    Addr result = sizeof(RegFile::reg_t) * numDtuRegs;

    result += static_cast<Addr>(reg) * sizeof(RegFile::reg_t);

    return result;
}

Addr
DtuAccel::getRegAddr(unsigned reg, unsigned epid)
{
    Addr result = sizeof(RegFile::reg_t) * (numDtuRegs + numCmdRegs);

    result += epid * numEpRegs * sizeof(RegFile::reg_t);

    result += reg * sizeof(RegFile::reg_t);

    return result;
}

bool
DtuAccel::CpuPort::recvTimingResp(PacketPtr pkt)
{
    dtuaccel.completeRequest(pkt);
    return true;
}

void
DtuAccel::CpuPort::recvReqRetry()
{
    dtuaccel.recvRetry();
}

DtuAccel::DtuAccel(const DtuAccelParams *p)
  : MemObject(p),
    system(p->system),
    tickEvent(this),
    chunkSize(system->cacheLineSize()),
    maxDataSize(p->max_data_size),
    port("port", this),
    masterId(system->getMasterId(name())),
    id(p->id),
    atomic(system->isAtomicMode()),
    reg_base(p->regfile_base_addr),
    retryPkt(nullptr),
    connector()
{
    DTUMemory *sys = dynamic_cast<DTUMemory*>(system);
    haveVM = !sys->hasMem(id);
    // if we don't have VM, we have an SPM, which supports larger chunks
    if (!haveVM)
        chunkSize = maxDataSize;

    // kick things into action
    schedule(tickEvent, curTick());
}

BaseMasterPort &
DtuAccel::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return MemObject::getMasterPort(if_name, idx);
}

bool
DtuAccel::sendPkt(PacketPtr pkt)
{
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
DtuAccel::recvRetry()
{
    assert(retryPkt);
    if (port.sendTimingReq(retryPkt))
        retryPkt = nullptr;
}

PacketPtr
DtuAccel::createPacket(Addr paddr,
                       size_t size,
                       MemCmd cmd = MemCmd::WriteReq)
{
    return createPacket(paddr, new uint8_t[size], size, cmd);
}

PacketPtr
DtuAccel::createPacket(Addr paddr,
                       const void *data,
                       size_t size,
                       MemCmd cmd = MemCmd::WriteReq)
{
    Request::Flags flags;

    auto req = new Request(paddr, size, flags, masterId);
    req->setContext(id);

    auto pkt = new Packet(req, cmd);
    pkt->dataDynamic(data);

    return pkt;
}

PacketPtr
DtuAccel::createDtuRegPkt(Addr reg,
                          RegFile::reg_t value,
                          MemCmd cmd = MemCmd::WriteReq)
{
    auto pkt = createPacket(reg_base + reg, sizeof(RegFile::reg_t), cmd);
    *pkt->getPtr<RegFile::reg_t>() = value;
    return pkt;
}

PacketPtr
DtuAccel::createDtuCmdPkt(uint64_t cmd,
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

    Dtu::Command::Bits cmdreg = 0;
    cmdreg.opcode = static_cast<RegFile::reg_t>(cmd);
    cmdreg.epid = epid;

    RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
    regs[0] = cmdreg;
    regs[1] = 0;
    regs[2] = data;
    regs[3] = size;
    regs[4] = off;
    regs[5] = EP_SYSR;
    return pkt;
}

void
DtuAccel::freePacket(PacketPtr pkt)
{
    delete pkt->req;
    // the packet will delete the data
    delete pkt;
}

std::string
DtuAccel::SyscallSM::stateName() const
{
    const char *names[] =
    {
        "SEND", "WAIT", "FETCH", "READ_ADDR", "ACK"
    };
    return names[static_cast<size_t>(state)];
}

PacketPtr
DtuAccel::SyscallSM::tick()
{
    PacketPtr pkt = nullptr;

    switch(state)
    {
        case State::SYSC_SEND:
        {
            pkt = accel->createDtuCmdPkt(Dtu::Command::SEND | (EP_SYSS << 4),
                                         MSG_ADDR,
                                         syscallSize,
                                         0);
            break;
        }
        case State::SYSC_WAIT:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = accel->createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::SYSC_FETCH:
        {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            uint64_t value = Dtu::Command::FETCH_MSG | (EP_SYSR << 4);
            pkt = accel->createDtuRegPkt(regAddr, value, MemCmd::WriteReq);
            break;
        }
        case State::SYSC_READ_ADDR:
        {
            Addr regAddr = getRegAddr(CmdReg::OFFSET);
            pkt = accel->createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::SYSC_ACK:
        {
            pkt = accel->createDtuCmdPkt(Dtu::Command::ACK_MSG | (EP_SYSR << 4),
                                         0,
                                         0,
                                         replyAddr);
            break;
        }
    }

    return pkt;
}

bool
DtuAccel::SyscallSM::handleMemResp(PacketPtr pkt)
{
    switch(state)
    {
        case State::SYSC_SEND:
        {
            state = State::SYSC_WAIT;
            break;
        }
        case State::SYSC_WAIT:
        {
            RegFile::reg_t reg = *pkt->getConstPtr<RegFile::reg_t>();
            if ((reg & 0xF) == 0)
                state = State::SYSC_FETCH;
            break;
        }
        case State::SYSC_FETCH:
        {
            state = State::SYSC_READ_ADDR;
            break;
        }
        case State::SYSC_READ_ADDR:
        {
            const RegFile::reg_t *regs = pkt->getConstPtr<RegFile::reg_t>();
            if(regs[0])
            {
                replyAddr = regs[0];
                state = State::SYSC_ACK;
            }
            else
                state = State::SYSC_FETCH;
            break;
        }
        case State::SYSC_ACK:
        {
            return true;
        }
    }

    return false;
}

std::string
DtuAccel::YieldSM::stateName() const
{
    std::ostringstream os;
    static const char *names[] =
    {
        "CHECK", "WAIT", "REPORT", "SYSCALL", "SLEEP"
    };
    os << names[static_cast<size_t>(state)];
    if (state == YLD_SYSCALL)
        os << ":" << syscsm->stateName();
    return os.str();
}

PacketPtr
DtuAccel::YieldSM::tick()
{
    PacketPtr pkt = nullptr;

    switch(state)
    {
        case State::YLD_CHECK:
        {
            pkt = accel->createPacket(RCTMUX_YIELD, sizeof(uint64_t), MemCmd::ReadReq);
            break;
        }
        case State::YLD_WAIT:
        {
            yieldStart = accel->curCycle();
            pkt = accel->createDtuCmdPkt(Dtu::Command::SLEEP, 0, 0, report);
            break;
        }
        case State::YLD_REPORT:
        {
            syscall.opcode = 10;   /* VPE_CTRL */
            syscall.vpe_sel = 0;   /* self */
            syscall.op = 2;        /* VCTRL_YIELD */
            syscall.arg = 0;       /* unused */

            pkt = accel->createPacket(MSG_ADDR, sizeof(syscall), MemCmd::WriteReq);
            memcpy(pkt->getPtr<void>(), &syscall, sizeof(syscall));
            break;
        }
        case State::YLD_SYSCALL:
        {
            pkt = syscsm->tick();
            break;
        }
        case State::YLD_SLEEP:
        {
            pkt = accel->createDtuCmdPkt(Dtu::Command::SLEEP, 0, 0, 0);
            break;
        }
    }

    return pkt;
}

bool
DtuAccel::YieldSM::handleMemResp(PacketPtr pkt)
{
    switch(state)
    {
        case State::YLD_CHECK:
        {
            report = *pkt->getConstPtr<uint64_t>();
            if (report > 0)
                state = State::YLD_WAIT;
            else
                state = State::YLD_SLEEP;
            break;
        }
        case State::YLD_WAIT:
        {
            if (accel->curCycle() < yieldStart + report)
                return true;

            state = State::YLD_REPORT;
            break;
        }
        case State::YLD_REPORT:
        {
            syscsm->start(sizeof(syscall));
            state = State::YLD_SYSCALL;
            break;
        }
        case State::YLD_SYSCALL:
        {
            if(syscsm->handleMemResp(pkt))
                state = State::YLD_SLEEP;
            break;
        }
        case State::YLD_SLEEP:
        {
            return true;
        }
    }

    return false;
}
