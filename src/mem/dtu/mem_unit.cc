/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (c) 2015, Nils Asmussen
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

#include "debug/Dtu.hh"
#include "debug/DtuBuf.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuSysCalls.hh"
#include "mem/dtu/mem_unit.hh"
#include "mem/dtu/xfer_unit.hh"
#include "mem/dtu/noc_addr.hh"

void
MemoryUnit::regStats()
{
    readBytes
        .init(8)
        .name(dtu.name() + ".mem.readBytes")
        .desc("Sent read requests (in bytes)")
        .flags(Stats::nozero);
    writtenBytes
        .init(8)
        .name(dtu.name() + ".mem.writtenBytes")
        .desc("Sent write requests (in bytes)")
        .flags(Stats::nozero);
    receivedBytes
        .init(8)
        .name(dtu.name() + ".mem.receivedBytes")
        .desc("Received read/write requests (in bytes)")
        .flags(Stats::nozero);
    wrongVPE
        .name(dtu.name() + ".mem.wrongVPE")
        .desc("Number of received requests that targeted the wrong VPE")
        .flags(Stats::nozero);
}

void
MemoryUnit::startRead(const Dtu::Command& cmd)
{
    MemEp ep = dtu.regs().getMemEp(cmd.arg);
    Addr rwBarrier = dtu.regs().get(DtuReg::RW_BARRIER);

    Addr localAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr requestSize = dtu.regs().get(CmdReg::DATA_SIZE);
    Addr offset = dtu.regs().get(CmdReg::OFFSET);

    readBytes.sample(requestSize);

    requestSize = std::min(dtu.maxNocPacketSize, requestSize);
    if (requestSize == 0)
        return;

    DPRINTFS(Dtu, (&dtu),
        "\e[1m[rd -> %u]\e[0m at %#018lx+%#lx with EP%u into %#018lx:%lu\n",
        ep.targetCore, ep.remoteAddr, offset,
        cmd.arg, localAddr, requestSize);

    // TODO error handling
    assert(localAddr < rwBarrier);
    assert(localAddr + requestSize <= rwBarrier);
    assert(ep.flags & Dtu::MemoryFlags::READ);
    assert(requestSize + offset >= requestSize);
    assert(requestSize + offset <= ep.remoteSize);

    Addr nocAddr = NocAddr(ep.targetCore,
                           ep.remoteAddr + offset).getAddr();
    auto pkt = dtu.generateRequest(nocAddr,
                                   requestSize,
                                   MemCmd::ReadReq);

    dtu.sendNocRequest(Dtu::NocPacketType::READ_REQ,
                       pkt,
                       ep.vpeId,
                       cmd.flags,
                       dtu.commandToNocRequestLatency);
}

void
MemoryUnit::startWrite(const Dtu::Command& cmd)
{
    MemEp ep = dtu.regs().getMemEp(cmd.arg);

    Addr localAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr requestSize = dtu.regs().get(CmdReg::DATA_SIZE);
    Addr offset = dtu.regs().get(CmdReg::OFFSET);

    writtenBytes.sample(requestSize);

    requestSize = std::min(dtu.maxNocPacketSize, requestSize);
    if (requestSize == 0)
        return;

    DPRINTFS(Dtu, (&dtu),
        "\e[1m[wr -> %u]\e[0m at %#018lx+%#lx with EP%u from %#018lx:%lu\n",
        ep.targetCore, ep.remoteAddr, offset,
        cmd.arg, localAddr, requestSize);

    // TODO error handling
    assert(ep.flags & Dtu::MemoryFlags::WRITE);
    assert(requestSize + offset >= requestSize);
    assert(requestSize + offset <= ep.remoteSize);

    uint flags = (cmd.flags & Dtu::Command::NOPF)
                 ? XferUnit::XferFlags::NOPF
                 : 0;

    dtu.startTransfer(Dtu::TransferType::LOCAL_READ,
                      NocAddr(ep.targetCore, ep.remoteAddr + offset),
                      localAddr,
                      requestSize,
                      NULL,
                      ep.vpeId,
                      NULL,
                      Cycles(0),
                      flags);
}

void
MemoryUnit::readComplete(const Dtu::Command& cmd, PacketPtr pkt, Dtu::Error error)
{
    dtu.printPacket(pkt);

    Addr localAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr requestSize = dtu.regs().get(CmdReg::DATA_SIZE);

    requestSize -= pkt->getSize();

    // since the transfer is done in steps, we can start after the header
    // delay here
    Cycles delay = dtu.ticksToCycles(pkt->headerDelay);

    if (error != Dtu::Error::NONE)
    {
        dtu.scheduleFinishOp(delay, error);
        return;
    }

    uint flags = (cmd.flags & Dtu::Command::NOPF)
                 ? XferUnit::XferFlags::NOPF
                 : 0;

    dtu.startTransfer(Dtu::TransferType::LOCAL_WRITE,
                      // remote address is irrelevant
                      NocAddr(0, 0),
                      localAddr,
                      pkt->getSize(),
                      pkt,
                      0,
                      NULL,
                      delay,
                      flags);
}

void
MemoryUnit::writeComplete(const Dtu::Command& cmd, PacketPtr pkt, Dtu::Error error)
{
    Addr requestSize = dtu.regs().get(CmdReg::DATA_SIZE);

    // error, write finished or if requestSize < pkt->getSize(), it was a msg
    if (error != Dtu::Error::NONE || requestSize <= pkt->getSize())
    {
        // we don't need to pay the payload delay here because the message
        // basically has no payload since we only receive an ACK back for
        // writing
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
        dtu.scheduleFinishOp(delay, error);
    }

    dtu.freeRequest(pkt);
}

void
MemoryUnit::recvFunctionalFromNoc(PacketPtr pkt)
{
    // set the local address
    pkt->setAddr(NocAddr(pkt->getAddr()).offset);

    dtu.sendFunctionalMemRequest(pkt);
}

Dtu::Error
MemoryUnit::recvFromNoc(PacketPtr pkt, uint vpeId, uint flags)
{
    NocAddr addr(pkt->getAddr());

    DPRINTFS(Dtu, (&dtu), "\e[1m[%s <- ?]\e[0m %#018lx:%lu\n",
        pkt->isWrite() ? "wr" : "rd",
        addr.offset,
        pkt->getSize());

    if (pkt->isWrite())
        dtu.printPacket(pkt);

    receivedBytes.sample(pkt->getSize());

    uint16_t ourVpeId = dtu.regs().get(DtuReg::VPE_ID);
    if (vpeId != ourVpeId)
    {
        DPRINTFS(Dtu, (&dtu),
            "Received memory request for VPE %u, but VPE %u is running\n",
            vpeId, ourVpeId);

        wrongVPE++;

        dtu.sendNocResponse(pkt);
        return Dtu::Error::VPE_GONE;
    }

    if (addr.offset >= dtu.regFileBaseAddr)
    {
        pkt->setAddr(addr.offset);

        dtu.forwardRequestToRegFile(pkt, false);

        // as this is synchronous, we can restore the address right away
        pkt->setAddr(addr.getAddr());
    }
    else
    {
        // the same as above: the transfer happens piece by piece and we can
        // start after the header
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;

        auto type = pkt->isWrite() ? Dtu::TransferType::REMOTE_WRITE
                                   : Dtu::TransferType::REMOTE_READ;
        uint xflags = (flags & Dtu::Command::NOPF) ? XferUnit::XferFlags::NOPF
                                                   : 0;
        dtu.startTransfer(type,
                          // remote address is irrelevant
                          NocAddr(0, 0),
                          // other remote is our local
                          addr.offset,
                          pkt->getSize(),
                          pkt,
                          0,
                          NULL,
                          delay,
                          xflags);
    }

    return Dtu::Error::NONE;
}
