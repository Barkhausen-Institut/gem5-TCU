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

const Addr DtuAccel::RCTMUX_YIELD      = 0x5ff0;
const Addr DtuAccel::RCTMUX_FLAGS      = 0x5ff8;

Addr
DtuAccel::getRegAddr(DtuReg reg)
{
    return static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

Addr
DtuAccel::getRegAddr(ReqReg reg)
{
    return DtuTlb::PAGE_SIZE + static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
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
DtuAccel::createDtuCmdPkt(Dtu::Command::Opcode cmd,
                          unsigned epid,
                          uint64_t data,
                          uint64_t size,
                          uint64_t arg,
                          uint64_t reply_label)
{
    static_assert(static_cast<int>(CmdReg::COMMAND) == 0, "");
    static_assert(static_cast<int>(CmdReg::ABORT) == 1, "");
    static_assert(static_cast<int>(CmdReg::DATA) == 2, "");
    static_assert(static_cast<int>(CmdReg::OFFSET) == 3, "");
    static_assert(static_cast<int>(CmdReg::REPLY_LABEL) == 4, "");

    auto pkt = createPacket(reg_base + getRegAddr(CmdReg::COMMAND),
                            sizeof(RegFile::reg_t) * 5,
                            MemCmd::WriteReq);

    Dtu::Command::Bits cmdreg = 0;
    cmdreg.opcode = static_cast<RegFile::reg_t>(cmd);
    cmdreg.epid = epid;
    cmdreg.arg = arg;

    RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
    regs[0] = cmdreg;
    regs[1] = 0;
    regs[2] = DataReg(data, size).value();
    regs[3] = 0;
    regs[4] = reply_label;
    return pkt;
}

void
DtuAccel::freePacket(PacketPtr pkt)
{
    delete pkt->req;
    // the packet will delete the data
    delete pkt;
}
