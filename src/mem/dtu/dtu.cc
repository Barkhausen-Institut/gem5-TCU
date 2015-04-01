/*
 * Copyright (c) 2015, Christian Menard
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 */

#include "mem/dtu/dtu.hh"
#include "debug/Dtu.hh"

Dtu::Dtu(const DtuParams *p)
  : MemObject(p),
    cpuBaseAddr(p->cpu_base_addr),
    dtuAddr(p->dtu_addr),
    dtuAddrBits(p->dtu_addr_bits),
    cpu("cpu", *this),
    scratchpad("scratchpad", *this),
    master("master", *this),
    slave("slave", *this)
{ }

void
Dtu::init()
{
    MemObject::init();

    assert(cpu.isConnected());
    assert(scratchpad.isConnected());
    assert(master.isConnected());
    assert(slave.isConnected());

    cpu.sendRangeChange();
    slave.sendRangeChange();
}

BaseMasterPort&
Dtu::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "scratchpad")
        return scratchpad;
    else if (if_name == "master")
        return master;
    else
        return MemObject::getMasterPort(if_name, idx);
}

BaseSlavePort&
Dtu::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu")
        return cpu;
    else if (if_name == "slave")
        return slave;
    else
        return MemObject::getSlavePort(if_name, idx);
}

Tick
Dtu::handleCpuRequest(PacketPtr pkt)
{
    // for now simply forward all requests to the scratchpad

    DPRINTF(Dtu, "Received %s at address 0x%x\n",
            pkt->isWrite() ? "write" : "read",
            pkt->getAddr());

    Addr addr = pkt->getAddr();
    addr &= 0x0fffffff;
    pkt->setAddr(addr);

    DPRINTF(Dtu, "Forward to scratchpad at address 0x%x\n",
            pkt->getAddr());

    scratchpad.sendAtomic(pkt);

    // TODO
    Tick dtuDelay = 0;

    Tick totalDelay = pkt->headerDelay + pkt->payloadDelay + dtuDelay;

    /*
     * The SimpleTimingPort already pays for the delay returned by recvAtomic
     *  -> reset the packet delay
     *
     * XXX I'm not sure if this is the right way to go. However, it seems
     *     better than simply ignoring the packet's delays as it is done for
     *     instance in SimpleMemory.
     */
    pkt->headerDelay  = 0;
    pkt->payloadDelay = 0;

    return totalDelay;
}

bool
Dtu::DtuScratchpadPort::recvTimingResp(PacketPtr pkt)
{
    panic("Did not expect a TimingResp!");
    return true;
}

void
Dtu::DtuScratchpadPort::recvReqRetry()
{
    panic("Did not expect a ReqRetry!");
}

AddrRangeList
Dtu::DtuCpuPort::getAddrRanges() const
{
    AddrRangeList ranges;
    auto range = AddrRange(dtu.cpuBaseAddr,
                           dtu.cpuBaseAddr + dtu.size - 1);
    ranges.push_back(range);
    return ranges;
}

Tick
Dtu::DtuCpuPort::recvAtomic(PacketPtr pkt)
{
    return dtu.handleCpuRequest(pkt);
}

bool
Dtu::DtuMasterPort::recvTimingResp(PacketPtr pkt)
{
    panic("Did not expect a TimingResp!");
    return true;
}

void
Dtu::DtuMasterPort::recvReqRetry()
{
    panic("Did not expect a ReqRetry!");
}

AddrRangeList
Dtu::DtuSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;

    // XXX assume 64 bit address width
    Addr baseAddr = dtu.dtuAddr << (64 - dtu.dtuAddrBits);
    Addr size = 1UL << (64 - dtu.dtuAddrBits);

    auto range = AddrRange(baseAddr, baseAddr + size - 1);
    ranges.push_back(range);
    return ranges;
}

Tick
Dtu::DtuSlavePort::recvAtomic(PacketPtr pkt)
{
    panic("Dtu::recvAtomic() not yet implemented");
    return 0;
}

void
Dtu::DtuSlavePort::recvFunctional(PacketPtr pkt)
{
    panic("Dtu::recvFunctional() not yet implemented");
}

bool
Dtu::DtuSlavePort::recvTimingReq(PacketPtr pkt)
{
    panic("Dtu::recvTimingReq() not yet implemented");
}

void
Dtu::DtuSlavePort::recvRespRetry()
{
    panic("Dtu::recvRespRetry() not yet implemented");
}

Dtu*
DtuParams::create()
{
    return new Dtu(this);
}
