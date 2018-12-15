/*
 * Copyright (c) 2017, Georg Kotheimer
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

#include "dev/dtu/pci_proxy.hh"

#include "base/trace.hh"
#include "debug/DtuPciProxy.hh"
#include "debug/DtuPciProxyCmd.hh"
#include "debug/DtuPciProxyDevMem.hh"
#include "debug/DtuPciProxyDma.hh"
#include "debug/DtuPciProxyInt.hh"
#include "dev/pci/pcireg.h"
#include "sim/system.hh"

const unsigned DtuPciProxy::EP_INT = 7;
const unsigned DtuPciProxy::EP_DMA = 8;
const Addr DtuPciProxy::REG_ADDR = 0x4000;
const Addr DtuPciProxy::INT_ADDR = 0x10000000;
const Addr DtuPciProxy::DMA_ADDR = 0x20000000;

PacketPtr
DtuPciProxy::createPacket(
    Addr paddr, size_t size, MemCmd cmd = MemCmd::WriteReq)
{
    return createPacket(paddr, new uint8_t[size], size, cmd);
}

PacketPtr
DtuPciProxy::createPacket(
    Addr paddr, const void* data, size_t size, MemCmd cmd = MemCmd::WriteReq)
{
    Request::Flags flags;

    auto req = std::make_shared<Request>(paddr, size, flags, masterId);
    req->setContext(id);

    auto pkt = new Packet(req, cmd);
    pkt->dataDynamic(data);

    return pkt;
}

void
DtuPciProxy::freePacket(PacketPtr pkt)
{
    // the packet will delete the data
    delete pkt;
}

Addr
DtuPciProxy::getRegAddr(DtuReg reg)
{
    return static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

PacketPtr
DtuPciProxy::createDtuRegPkt(
    Addr reg, RegFile::reg_t value, MemCmd cmd = MemCmd::WriteReq)
{
    auto pkt = createPacket(dtuRegBase + reg, sizeof(RegFile::reg_t), cmd);
    *pkt->getPtr<RegFile::reg_t>() = value;
    return pkt;
}

Addr
DtuPciProxy::getRegAddr(CmdReg reg)
{
    Addr result = sizeof(RegFile::reg_t) * numDtuRegs;

    result += static_cast<Addr>(reg) * sizeof(RegFile::reg_t);

    return result;
}

PacketPtr
DtuPciProxy::createDtuCmdPkt(Dtu::Command::Opcode cmd, unsigned epid,
    uint64_t data, uint64_t size, uint64_t arg)
{
    static_assert(static_cast<int>(CmdReg::COMMAND) == 0, "");
    static_assert(static_cast<int>(CmdReg::ABORT) == 1, "");
    static_assert(static_cast<int>(CmdReg::DATA) == 2, "");

    auto pkt = createPacket(dtuRegBase + getRegAddr(CmdReg::COMMAND),
        sizeof(RegFile::reg_t) * 3, MemCmd::WriteReq);

    Dtu::Command::Bits cmdreg = 0;
    cmdreg.opcode = static_cast<RegFile::reg_t>(cmd);
    cmdreg.epid = epid;
    cmdreg.arg = arg;

    RegFile::reg_t* regs = pkt->getPtr<RegFile::reg_t>();
    regs[0] = cmdreg;
    regs[1] = 0;
    regs[2] = DataReg(data, size).value();
    return pkt;
}

Addr
DtuPciProxy::encodePciAddress(PciBusAddr const& busAddr, Addr offset)
{
    Addr addr = insertBits(0, 15, 8, busAddr.bus);
    addr = insertBits(addr, 7, 3, busAddr.dev);
    addr = insertBits(addr, 2, 0, busAddr.func);
    return (addr << 8) | (offset & mask(8));
}

PacketPtr
DtuPciProxy::createPciConfigPacket(
    PciBusAddr busAddr, Addr offset, const void* data, size_t size, MemCmd cmd)
{
    Request::Flags flags;

    Addr addr = encodePciAddress(busAddr, offset);
    auto req = std::make_shared<Request>(addr, size, flags, masterId);

    auto pkt = new Packet(req, cmd);
    pkt->dataStatic(data);

    return pkt;
}

DtuPciProxy::DtuPciProxy(const DtuPciProxyParams* p)
    : MemObject(p),
      dtuMasterPort(name() + ".dtu_master_port", this),
      dtuSlavePort(name() + ".dtu_slave_port", this),
      pioPort(name() + ".pio_port", this),
      dmaPort(name() + ".dma_port", this),
      masterId(p->system->getMasterId(this, name())),
      id(p->id),
      dtuRegBase(p->dtu_regfile_base_addr),
      deviceBusAddr(0, 0, 0),
      tickEvent(this),
      cmdSM(this),
      cmdRunning(false),
      interruptPending(false),
      pendingDmaReq(nullptr),
      dmaRetry(false)
{
}

BaseMasterPort&
DtuPciProxy::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "dtu_master_port")
        return dtuMasterPort;
    else if (if_name == "pio_port")
        return pioPort;
    else
        return MemObject::getMasterPort(if_name, idx);
}

BaseSlavePort&
DtuPciProxy::getSlavePort(const std::string& if_name, PortID idx)
{
    if (if_name == "dtu_slave_port")
        return dtuSlavePort;
    else if (if_name == "dma_port")
        return dmaPort;
    else
        return MemObject::getSlavePort(if_name, idx);
}

void
DtuPciProxy::init()
{
    fatal_if(!findDevice(), "Failed to find a device to proxy.");

    dtuSlavePort.sendRangeChange();
    dmaPort.sendRangeChange();
}

bool
DtuPciProxy::findDevice()
{
    DPRINTF(DtuPciProxy, "Enumerating devices...\n");

    for (size_t bus = 0; bus < 256; bus++) {
        for (size_t dev = 0; dev < 32; dev++) {
            uint16_t vendor = 0xFFFF;
            pciHost->read(createPciConfigPacket(PciBusAddr(bus, dev, 0),
                PCI_VENDOR_ID, &vendor, 2, MemCmd::ReadReq));

            if (vendor != 0xFFFF) {
                DPRINTF(DtuPciProxy, "Found device with vendor id: %04x\n",
                    vendor);
                deviceBusAddr = PciBusAddr(bus, dev, 0);
                return true;
            }
        }
    }

    return false;
}

void
DtuPciProxy::executeCommand(PacketPtr cmdPkt)
{
    assert(!cmdRunning);

    DPRINTF(DtuPciProxyCmd, "Execute DTU command.\n");
    cmdRunning = true;
    cmdSM.executeCommand(cmdPkt);
}

void
DtuPciProxy::commandExecutionFinished()
{
    cmdRunning = false;
    DPRINTF(DtuPciProxyCmd, "Finished DTU command execution.\n");

    if (pendingDmaReq && pendingDmaReq->isResponse()) {
        DPRINTF(DtuPciProxyDma,
            "Send response for DMA write request to device.\n");
        dmaPort.schedTimingResp(pendingDmaReq, clockEdge(Cycles(1)));
        pendingDmaReq = nullptr;
    }

    if (interruptPending) {
        sendInterruptCmd();
    } else if (pendingDmaReq) {
        sendDmaCmd();
    } else if (dmaRetry) {
        DPRINTF(DtuPciProxyDma, "Send DMA retry to device.\n");
        dmaRetry = false;
        dmaPort.sendRetryReq();
    }
}

void
DtuPciProxy::signalInterrupt()
{
    DPRINTF(DtuPciProxyInt,
        "Device signaled interrupt (pending: %s, cmdRunning: %s)\n",
        interruptPending ? "true" : "false", cmdRunning ? "true" : "false");

    interruptPending = true;
    if (!cmdRunning)
        sendInterruptCmd();
}

void
DtuPciProxy::sendInterruptCmd()
{
    DPRINTF(
        DtuPciProxyInt, "Send interrupt message using endpoint %u\n", EP_INT);

    PacketPtr cmdPkt
        = createDtuCmdPkt(Dtu::Command::SEND, EP_INT, INT_ADDR, 0x4, 0);
    interruptPending = false;
    executeCommand(cmdPkt);
}

void
DtuPciProxy::handleInterruptMessageContent(PacketPtr pkt)
{
    assert(pkt->needsResponse());
    assert(pkt->isRead());

    pkt->makeResponse();
    memset(pkt->getPtr<uint8_t>(), 0, pkt->getSize());
    dtuSlavePort.schedTimingResp(pkt, clockEdge(Cycles(1)));
}

void
DtuPciProxy::forwardAccessToDeviceMem(PacketPtr pkt)
{
    assert(pkt->getAddr() >= REG_ADDR);
    Addr offset = pkt->getAddr() - REG_ADDR;

    DPRINTF(DtuPciProxyDevMem,
        "Forward %s access at %llx (%llu) to device memory at %llx\n",
        pkt->isWrite() ? "write" : "read", pkt->getAddr(), pkt->getSize(),
        offset);
    pkt->setAddr(pciHost->memAddr(deviceBusAddr, offset));

    pioPort.schedTimingReq(pkt, clockEdge(Cycles(1)));
}

void
DtuPciProxy::completeAccessToDeviceMem(PacketPtr pkt)
{
    DPRINTF(DtuPciProxyDevMem,
        "Send response for device memory %s access at %llx.\n",
        pkt->isWrite() ? "write" : "read", pkt->getAddr());

    // DTU always accepts responses
    dtuSlavePort.schedTimingResp(pkt, clockEdge(Cycles(1)));
}

bool
DtuPciProxy::handleDmaRequest(PacketPtr pkt)
{
    assert(!dmaRetry);

    DPRINTF(DtuPciProxyDma,
        "Received DMA request from device (pending: %s, cmdRunning: %s)\n",
        pendingDmaReq ? "true" : "false", cmdRunning ? "true" : "false");

    if (pendingDmaReq || cmdRunning) {
        dmaRetry = true;
        DPRINTF(DtuPciProxyDma, "Defer DMA request.\n");
        return false;
    }

    pendingDmaReq = pkt;
    sendDmaCmd();

    return true;
}

void
DtuPciProxy::sendDmaCmd()
{
    assert(pendingDmaReq);

    DPRINTF(DtuPciProxyDma,
        "Execute DMA request using endpoint %u: %s @ %llx with %llu bytes\n",
        EP_DMA, pendingDmaReq->cmdString(), pendingDmaReq->getAddr(),
        pendingDmaReq->getSize());

    // TODO: Validate offset lies within the memory endpoint's boundaries
    // Translate to DTU read/write command
    auto cmd
        = pendingDmaReq->isRead() ? Dtu::Command::READ : Dtu::Command::WRITE;
    PacketPtr cmdPkt = createDtuCmdPkt(cmd, EP_DMA, DMA_ADDR,
        pendingDmaReq->getSize(), pendingDmaReq->getAddr());
    executeCommand(cmdPkt);
}

void
DtuPciProxy::handleDmaContent(PacketPtr pkt)
{
    assert(pendingDmaReq);

    // Provide data for dma write
    if (pkt->isRead()) {
        DPRINTF(DtuPciProxyDma, "Send data for DMA write request to DTU.\n");

        pkt->makeResponse();
        pkt->setData(pendingDmaReq->getPtr<uint8_t>());
        DDUMP(DtuPciProxyDma, pkt->getPtr<uint8_t>(), pkt->getSize());

        dtuSlavePort.schedTimingResp(pkt, clockEdge(Cycles(1)));

        pendingDmaReq->makeResponse();
    } else {
        DPRINTF(
            DtuPciProxyDma, "Receive data for DMA read request from DTU.\n");

        pendingDmaReq->makeResponse();
        pendingDmaReq->setData(pkt->getPtr<uint8_t>());
        DDUMP(DtuPciProxyDma, pendingDmaReq->getPtr<uint8_t>(),
            pendingDmaReq->getSize());

        DPRINTF(
            DtuPciProxyDma, "Send response for DMA read request to device.\n");

        dmaPort.schedTimingResp(pendingDmaReq, clockEdge(Cycles(1)));
        pendingDmaReq = nullptr;

        if (pkt->needsResponse()) {
            pkt->makeResponse();
            dtuSlavePort.schedTimingResp(pkt, clockEdge(Cycles(1)));
        }
    }
}

void
DtuPciProxy::tick()
{
    cmdSM.tick();
}

std::string
DtuPciProxy::CommandSM::stateName() const
{
    const char* names[] = { "IDLE", "SEND", "WAIT" };
    return names[static_cast<size_t>(state)];
}

void
DtuPciProxy::CommandSM::executeCommand(PacketPtr cmdPkt)
{
    assert(isIdle());
    assert(!cmd);
    state = CMD_SEND;
    cmd = cmdPkt;
    tick();
}

void
DtuPciProxy::CommandSM::tick()
{
    PacketPtr pkt = nullptr;

    switch (state) {
        case State::CMD_IDLE: {
            pciProxy->commandExecutionFinished();
            break;
        }
        case State::CMD_SEND: {
            pkt = cmd;
            break;
        }
        case State::CMD_WAIT: {
            Addr regAddr = getRegAddr(CmdReg::COMMAND);
            pkt = pciProxy->createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
    }

    if (pkt != nullptr) {
        pciProxy->dtuMasterPort.schedTimingReq(
            pkt, pciProxy->clockEdge(Cycles(1)));
    }
}

void
DtuPciProxy::CommandSM::handleMemResp(PacketPtr pkt)
{
    RequestPtr req = pkt->req;

    Cycles delay(1);
    if (pkt->isError()) {
        warn("%s access failed at %#x\n", pkt->isWrite() ? "Write" : "Read",
            req->getPaddr());
    } else {
        switch (state) {
            case State::CMD_IDLE: {
                assert(false);
                break;
            }
            case State::CMD_SEND: {
                cmd = nullptr;
                state = State::CMD_WAIT;
                break;
            }
            case State::CMD_WAIT: {
                RegFile::reg_t reg = *pkt->getConstPtr<RegFile::reg_t>();
                if ((reg & 0xF) == 0)
                    state = State::CMD_IDLE;
                break;
            }
        }
    }

    freePacket(pkt);

    // kick things into action again
    pciProxy->schedule(pciProxy->tickEvent, pciProxy->clockEdge(delay));
}

bool
DtuPciProxy::DtuMasterPort::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());

    pciProxy.cmdSM.handleMemResp(pkt);
    return true;
}

bool
DtuPciProxy::DtuSlavePort::recvTimingReq(PacketPtr pkt)
{
    if (pkt->getAddr() >= DMA_ADDR) {
        pciProxy.handleDmaContent(pkt);
    } else if (pkt->getAddr() >= INT_ADDR) {
        pciProxy.handleInterruptMessageContent(pkt);
    } else if (pkt->getAddr() >= REG_ADDR) {
        pciProxy.forwardAccessToDeviceMem(pkt);
    } else {
        warn("Received unexpected request at %llx\n", pkt->getAddr());
    }

    return true;
}

void
DtuPciProxy::DtuSlavePort::recvFunctional(PacketPtr pkt)
{
    panic("not implemented");
}

Tick
DtuPciProxy::DtuSlavePort::recvAtomic(PacketPtr pkt)
{
    panic("not implemented");

    return 0;
}

AddrRangeList
DtuPciProxy::DtuSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;
    // MEMCAP_END = 0x3fc00000
    ranges.push_back(AddrRange(0, 0x3fc00000));
    return ranges;
}

bool
DtuPciProxy::PioPort::recvTimingResp(PacketPtr pkt)
{
    pciProxy.completeAccessToDeviceMem(pkt);
    return true;
}

bool
DtuPciProxy::DmaPort::recvTimingReq(PacketPtr pkt)
{
    return pciProxy.handleDmaRequest(pkt);
}

void
DtuPciProxy::DmaPort::recvFunctional(PacketPtr pkt)
{
    panic("not implemented");
}

Tick
DtuPciProxy::DmaPort::recvAtomic(PacketPtr pkt)
{
    panic("not implemented");

    return 0;
}

AddrRangeList
DtuPciProxy::DmaPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, 0xffffffffffffffff));
    return ranges;
}

DtuPciProxy*
DtuPciProxyParams::create()
{
    return new DtuPciProxy(this);
}
