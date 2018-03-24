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

#include "dev/storage/dtuide/base_proxy.hh"

#include <iomanip>

#include "arch/x86/regs/misc.hh"
#include "arch/x86/x86_traits.hh"
#include "debug/BaseProxy.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/regfile.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"
#include "sim/dtu_memory.hh"

const unsigned BaseProxy::EP_SEND       = 8;
const unsigned BaseProxy::EP_REPLY      = 9;

const unsigned BaseProxy::PREP_PACKET_ADDR   = 0x10000000;
const unsigned BaseProxy::RESPOND_ADDR       = 0x10000001;
const unsigned BaseProxy::ACK_ADDR           = 0x100000FF;

static const char * names[] = {
            "IDLE",
            "SEND",
            "SEND_COMMAND",
            "WAIT",
            "FETCH",
            "READ_ADDR",
            "ACK"
        };

BaseProxy::BaseProxy(const BaseProxyParams *p)
  : MemObject(p),
    masterId(p->system->getMasterId(name())),
    id(p->id),
    atomic(p->system->isAtomicMode()),
    reg_base(p->regfile_base_addr),
    retryPkt(nullptr),
    tempPacket(nullptr),
    intAnswered(true),
    dev_port("dev_port", this),
    dtu_port("dtu_port", this, p->ranges),
    int_port("int_port", this),
    respPacketList(new std::list<PacketPtr>()),
    recvPacketList(new std::list<PacketPtr>()),
    system(p->system),
    tickEvent(this),
    memRespEvent(this),
    interruptSM(new InterruptSM(this)),
    con(nullptr)
{
    schedule(tickEvent, curTick());
    interruptSM->start(1);
}

BaseProxy *
BaseProxyParams::create()
{
    return new BaseProxy(this);
}

void
BaseProxy::init()
{
    MemObject::init();

    if (dtu_port.isConnected())
    {
        dtu_port.sendRangeChange();
    }
}

BaseMasterPort &
BaseProxy::getMasterPort(const std::string& if_name, PortID idx)
{
    DPRINTF(BaseProxy, "Name of the the port: %s\n", if_name);

    if (if_name == "dev_port")
        return dev_port;
    else if (if_name == "int_port")
        return int_port;
    else
        return MemObject::getMasterPort(if_name, idx);
}

BaseSlavePort &
BaseProxy::getSlavePort(const std::string& if_name, PortID idx)
{
    DPRINTF(BaseProxy, "Name of the supposed slave port: %s\n", if_name);
    if (if_name == "dtu_port")
        return dtu_port;
    else
        return MemObject::getSlavePort(if_name, idx);
}



PacketPtr
BaseProxy::createPacket(Addr paddr,
                       size_t size,
                       MemCmd cmd = MemCmd::WriteReq)
{
    return createPacket(paddr, new uint8_t[size], size, cmd);
}

PacketPtr
BaseProxy::createPacket(Addr paddr,
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
BaseProxy::createDtuRegPkt(Addr reg,
                          RegFile::reg_t value,
                          MemCmd cmd = MemCmd::WriteReq)
{
    auto pkt = createPacket(reg_base + reg, sizeof(RegFile::reg_t), cmd);
    *pkt->getPtr<RegFile::reg_t>() = value;
    return pkt;
}

PacketPtr
BaseProxy::createDtuCmdPkt(Dtu::Command::Opcode cmd,
                           unsigned epid,
                           uint64_t data,
                           uint64_t size,
                           uint64_t arg)
{
    static_assert(static_cast<int>(CmdReg::COMMAND) == 0, "");
    static_assert(static_cast<int>(CmdReg::ABORT) == 1, "");
    static_assert(static_cast<int>(CmdReg::DATA) == 2, "");

    auto pkt = createPacket(reg_base + getRegAddr(CmdReg::COMMAND),
                            sizeof(RegFile::reg_t) * 3,
                            MemCmd::WriteReq);

    Dtu::Command::Bits cmdreg = 0;
    cmdreg.opcode = static_cast<RegFile::reg_t>(cmd);
    cmdreg.epid = epid;
    cmdreg.arg = arg;

    RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
    regs[0] = cmdreg;
    regs[1] = 0;
    regs[2] = DataReg(data, size).value();
    return pkt;
}

void
BaseProxy::freePacket(PacketPtr pkt)
{
    delete pkt->req;
    delete pkt;
}

bool
BaseProxy::forwardToDevice(PacketPtr pkt)
{
    DPRINTF(BaseProxy, "Sending packet IDE addr 0x%x\n", pkt->getAddr());

    if (!dev_port.sendTimingReq(pkt)) {
        recvPacketList->push_back(pkt);
        this->recvRetry();
    }
    return true;
}

bool
BaseProxy::forwardFromDevice(PacketPtr pkt)
{

    DPRINTF(BaseProxy,
        "Sending packet DTU addr 0x%x\n", pkt->getAddr());

    if (!dtu_port.sendTimingResp(pkt)) {
        respPacketList->push_back(pkt);
        this->respRetry();
    }

    return true;
}

bool
BaseProxy::DeviceSidePort::recvTimingResp(PacketPtr pkt)
{
    if (baseproxy.forwardFromDevice(pkt)) {
        return true;
    }
    else {
        panic("Not handled!");
        return false;
    }
}

void
BaseProxy::DeviceSidePort::recvReqRetry()
{
    baseproxy.recvRetry();
}

void
BaseProxy::InterruptPort::recvReqRetry()
{
    panic("Request retry not intended for interrupt port");
}

bool
BaseProxy::InterruptPort::recvTimingResp(PacketPtr pkt)
{
    baseproxy.completeRequest(pkt);
    return true;
}

bool
BaseProxy::DtuSidePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTFS(BaseProxy, (&baseproxy), "Forwarding packet with address 0x%x\n",
        pkt->getAddr());

    /* This address range is designated for interaction with the state
     * machine. */
    if (pkt->getAddr() >= 0x10000000 && pkt->getAddr() <= 0x100000FF) {
        baseproxy.handleIntResp(pkt);
        return true;
    }
    else {
        return baseproxy.forwardToDevice(pkt);
    }
}

void
BaseProxy::DtuSidePort::recvRespRetry()
{
    baseproxy.respRetry();
}

Tick
BaseProxy::DtuSidePort::recvAtomic(PacketPtr pkt)
{
    panic("DtuSidePort::recvAtomic should not be callable!");
}

void
BaseProxy::DtuSidePort::recvFunctional(PacketPtr pkt)
{
    DPRINTFS(BaseProxy, (&baseproxy),
        "Functionally forwarding packet with address 0x%x\n",
        pkt->getAddr());

    if (pkt->getAddr() < 0xFF)
        baseproxy.dev_port.sendFunctional(pkt);
}

AddrRangeList
BaseProxy::DtuSidePort::getAddrRanges() const
{
    DPRINTFS(BaseProxy, (&baseproxy), "Getting address ranges...\n");
    return addrRangeList;
}

void
BaseProxy::recvRetry()
{
    assert(!recvPacketList->empty());
    while (recvPacketList->empty()) {
    //TODO: handle race conditions if multiple
        //threads want to retry message.
        if (dev_port.sendTimingReq(*(recvPacketList->begin()))) {
            recvPacketList->pop_front();
            return;
        }
    }
}

void
BaseProxy::respRetry()
{
    assert(!respPacketList->empty());
    while (respPacketList->empty()) { //TODO: race conditions?
        if (dtu_port.sendTimingResp(*(respPacketList->begin()))) {
            respPacketList->pop_front();
            return;
        }
    }
}

void
BaseProxy::handleIntResp(PacketPtr pkt)
{
    uint32_t data;

    switch(pkt->getAddr()) {
        case PREP_PACKET_ADDR: //deprecated
            pkt->makeResponse();
            data = 0xFF; // bogus
            pkt->set<uint32_t>(data);
            respPacketList->push_back(pkt);
            schedule(memRespEvent, curTick());
            break;
        case RESPOND_ADDR:
            data = 0xFFFF; //bogus
            intAnswered = false;
            *(pkt->getPtr<uint32_t>()) = data;
            if (pkt->needsResponse())
                pkt->makeResponse();
            else
                panic("Packet needs to require response!");
            DPRINTF(BaseProxy,"Sending response\n");
            tempPacket = pkt;
            respPacketList->push_back(pkt);
            schedule(memRespEvent, curTick());
            break;
        case ACK_ADDR:
            intAnswered = true;
            pkt->makeResponse();
            respPacketList->push_back(pkt);
            schedule(memRespEvent, curTick());
            break;
        default:
            panic("Answer for address 0x%x not implemented\n", pkt->getAddr());
    }

}

bool
BaseProxy::sendToDtu(PacketPtr pkt)
{
    assert(pkt!=nullptr);

    if (atomic)
    {
        panic("Sending atomically not intended");
        int_port.sendAtomic(pkt);
        return true;
    }

    if (!int_port.sendTimingReq(pkt))
    {
        panic("Failed to send packet from interrupt port");
        retryPkt = pkt;
        return false;
    }

    return true;
}

void
BaseProxy::tick()
{
    DPRINTF(BaseProxy, "[%s] tick\n", interruptSM->getStateName());
    PacketPtr pkt = interruptSM->tick();

    if (pkt != nullptr && !interruptSM->isIdle()) {
        DPRINTF(BaseProxy, "Sending pkt with addr 0x%x to DTU sided port\n",
            pkt->getAddr());
        sendToDtu(pkt);
    }
    else if (pkt == nullptr && !interruptSM->isIdle()) {
    // only schedule tick if no packet needs to be handled
        schedule(tickEvent, curTick());
    }
}

/* Unify this and respRetry */
void
BaseProxy::sendMemResponse()
{
    DPRINTF(BaseProxy, "Sending mem response asynchronously\n");
    while (!respPacketList->empty())
    {
        if (dtu_port.sendTimingResp(respPacketList->front())) {
            respPacketList->pop_front();
        }
    }

    DPRINTF(BaseProxy, "Response sent\n");
}

void
BaseProxy::completeRequest(PacketPtr pkt)
{
    DPRINTF(BaseProxy, "[%s] Completing request\n",
        interruptSM->getStateName());

    interruptSM->handleMemResp(pkt);
    freePacket(pkt);

    if (!interruptSM->isIdle() &&
        interruptSM->getState() != InterruptSM::State::FETCH)
        schedule(tickEvent, curTick());
}

Addr
BaseProxy::getRegAddr(DtuReg reg)
{
    return static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

Addr
BaseProxy::getRegAddr(CmdReg reg)
{
    Addr result = sizeof(RegFile::reg_t) * numDtuRegs;

    result += static_cast<Addr>(reg) * sizeof(RegFile::reg_t);

    DPRINTF(BaseProxy, "Address for reg %d is 0x%x\n",
        (Addr) reg, (Addr) result);
    return result;
}

Addr
BaseProxy::getRegAddr(unsigned reg, unsigned epid)
{
    Addr result = sizeof(RegFile::reg_t) * (numDtuRegs + numCmdRegs);

    result += epid * numEpRegs * sizeof(RegFile::reg_t);

    result += reg * sizeof(RegFile::reg_t);

    return result;
}

void
BaseProxy::postConsoleInt()
{
    DPRINTF(BaseProxy, "Trying to send interrupt!\n");
    panic("Console interrupt posting not handled");
}

void
BaseProxy::clearConsoleInt()
{
    DPRINTF(BaseProxy, "Trying to clear interrupt!\n");
    panic("Console interrupt clearing not handled");
}

void
BaseProxy::postPciInt(int line)
{
    DPRINTF(BaseProxy, "Trying to send PCI interrupt on line %d\n", line);
    interruptSM->setInterruptPending();

    schedule(tickEvent, curTick());
}

void
BaseProxy::clearPciInt(int line)
{
    DPRINTF(BaseProxy, "Trying to clear PCI interrupt on line %d\n", line);
    interruptSM->clearInterrupt();

}

void
BaseProxy::wakeup()
{
    DPRINTF(BaseProxy, "Proxy woken up, checking on DMA\n");
    schedule(tickEvent, curTick());
}

void
BaseProxy::reset()
{
    DPRINTF(BaseProxy,
            "Resetting the proxy and its state is not yet implemented\n");
}

void
BaseProxy::interruptFromConnector()
{
    DPRINTF(BaseProxy, "Interrupt from connector is not intended\n");
}

void
BaseProxy::setConnector(PCIConnector * con)
{
    this->con = con;
}

std::string
BaseProxy::InterruptSM::getStateName()
{
    return names[static_cast<uint>(state)];
}

void
BaseProxy::InterruptSM::setInterruptPending()
{
    interruptPending = true;
}

void
BaseProxy::InterruptSM::clearInterrupt()
{
    interruptPending = false;
}

/** This state machine follows quite a linear flow:
IDLE -> SEND_COMMAND -> WAIT -> FETCH -> READ_ADDR -> ACK
(SEND not used anymore, please also edit the name array if you edit the
enum)

This tick is called in the tick method of the base proxy class. It returns
a packet if the state is not IDLE. Otherwise the packet is then sent either
to the DTU or reads from the proxy itself again, since the proxy also acts
as memory for the DTU.
*/
PacketPtr
BaseProxy::InterruptSM::tick()
{
    PacketPtr pkt = nullptr;

    switch(state)
    {
        case State::IDLE:
        {
            if (interruptPending) {
                interruptPending = false;
                state = SEND_COMMAND;
            }
            break;
        }
        case State::SEND: //deprecated
        {
            pkt = proxy->createDtuRegPkt(PREP_PACKET_ADDR, //addr
                                  0xFFFF, //data
                                  MemCmd::WriteReq);
            break;
        }
        case State::SEND_COMMAND:
            pkt = proxy->createDtuCmdPkt(Dtu::Command::SEND,
                                  EP_SEND,
                                  RESPOND_ADDR, // addr
                                  sizeof(uint32_t), //msg size
                                  EP_REPLY); // reply EP
            break;
        case State::WAIT: // wait for answering interrupt
        {
            Addr regAddr = proxy->getRegAddr(CmdReg::COMMAND);
            pkt = proxy->createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::FETCH:
        {
            Addr regAddr = proxy->getRegAddr(CmdReg::COMMAND);
            uint64_t value = Dtu::Command::FETCH_MSG | (EP_REPLY << 4);
            pkt = proxy->createDtuRegPkt(regAddr, value, MemCmd::WriteReq);
            break;
        }
        case State::READ_ADDR:
        {
            Addr regAddr = proxy->getRegAddr(CmdReg::OFFSET);
            pkt = proxy->createDtuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
        case State::ACK:
        {
            pkt = proxy->createDtuCmdPkt(
                Dtu::Command::ACK_MSG,
                EP_REPLY,
                0,
                0,
                replyAddr);
            break;
        }
        default:
            panic("Not implemented, state: %s",getStateName());
    }

    return pkt;
}

bool
BaseProxy::InterruptSM::handleMemResp(PacketPtr pkt)
{

    RegFile::reg_t reg;
    const RegFile::reg_t *regs;

    switch(state) {
        case IDLE:
            panic("handleMemResp should not be called during IDLE");
        case SEND:
            state = SEND_COMMAND;
            break;
        case SEND_COMMAND:
            state = WAIT;
            break;
        case WAIT:
            reg = *pkt->getConstPtr<RegFile::reg_t>();
            if ((reg & 0xF) == 0) {// check if the remote VPE has updated
                                  // the register yet
                state = FETCH;
            }
            break;
        case FETCH:
            state = READ_ADDR;
            break;
        case READ_ADDR:
            regs = pkt->getConstPtr<RegFile::reg_t>();
            if (regs[0]) {
                replyAddr = regs[0];
                state = ACK;
            }
            else {
                state = FETCH;
            }
            break;
        case ACK:
            state = IDLE;
            break;
        default:
            panic("Not implemented, state: %s",getStateName());
            break;
    }
    return true;
}

bool
BaseProxy::InterruptSM::isIdle()
{
    return state == IDLE;
}

BaseProxy::InterruptSM::State
BaseProxy::InterruptSM::getState()
{
    return state;
}
