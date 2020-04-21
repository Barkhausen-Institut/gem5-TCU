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

#include "arch/x86/interrupts.hh"
#include "debug/TcuConnector.hh"
#include "mem/tcu/connector/x86.hh"
#include "mem/tcu/tcu.hh"
#include "cpu/simple/base.hh"
#include "sim/process.hh"

X86Connector::X86Connector(const X86ConnectorParams *p)
  : CoreConnector(p),
    irqPort("irq_master_port", this)
{
}

Port&
X86Connector::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "irq_master_port")
        return irqPort;
    else
        return SimObject::getPort(if_name, idx);
}

bool
X86Connector::IrqMasterPort::sendPacket(PacketPtr pkt)
{
    bool retry = !sendTimingReq(pkt);
    if (retry)
    {
        pending = pkt;
        return false;
    }
    return true;
}

bool
X86Connector::IrqMasterPort::recvTimingResp(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void
X86Connector::IrqMasterPort::recvReqRetry()
{
    assert(pending);
    if (sendTimingReq(pending))
        pending = nullptr;
}

void
X86Connector::doSetIrq(IRQ irq)
{
    const int APIC_ID = 0;

    X86ISA::TriggerIntMessage message = 0;
    message.deliveryMode = X86ISA::DeliveryMode::ExtInt;
    message.destination = APIC_ID;
    message.destMode = 0;   // physical
    message.trigger = 0;    // edge
    message.level = 0;      // unused?
    message.vector = 0x40 + irq;

    DPRINTF(TcuConnector,
            "Injecting IRQ %d (vector %d)\n", irq, message.vector);

    PacketPtr pkt = X86ISA::buildIntTriggerPacket(APIC_ID, message);
    irqPort.sendPacket(pkt);
}

X86Connector*
X86ConnectorParams::create()
{
    return new X86Connector(this);
}
