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
#include "debug/DtuDetail.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuSysCalls.hh"
#include "debug/DtuPower.hh"
#include "mem/dtu/mem_unit.hh"
#include "mem/dtu/noc_addr.hh"

void
MemoryUnit::startRead(const Dtu::Command& cmd)
{
    unsigned targetCoreId = dtu.regs().get(cmd.epId, EpReg::TGT_COREID);
    Addr localAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr requestSize = dtu.regs().get(CmdReg::DATA_SIZE);
    Addr offset = dtu.regs().get(CmdReg::OFFSET);
    Addr remoteAddr = dtu.regs().get(cmd.epId, EpReg::REQ_REM_ADDR);
    Addr remoteSize = dtu.regs().get(cmd.epId, EpReg::REQ_REM_SIZE);
    unsigned flags = dtu.regs().get(cmd.epId, EpReg::REQ_FLAGS);

    // we'll need that in readComplete
    continueEvent.cmd = cmd;
    continueEvent.read = true;

    requestSize = std::min(dtu.maxNocPacketSize, requestSize);
    if(requestSize == 0)
        return;

    DPRINTFS(Dtu, (&dtu), "\e[1m[rd -> %u]\e[0m at offset %#018lx with EP%u into %#018lx:%lu\n",
        targetCoreId, offset, cmd.epId, localAddr, requestSize);

    // TODO error handling
    assert(flags & Dtu::MemoryFlags::READ);
    assert(requestSize + offset >= requestSize);
    assert(requestSize + offset <= remoteSize);

    auto pkt = dtu.generateRequest(NocAddr(targetCoreId, 0, remoteAddr + offset).getAddr(),
                                   requestSize,
                                   MemCmd::ReadReq);

    dtu.sendNocRequest(Dtu::NocPacketType::READ_REQ,
                       pkt,
                       dtu.commandToNocRequestLatency);
}

void
MemoryUnit::startWrite(const Dtu::Command& cmd)
{
    unsigned targetCoreId = dtu.regs().get(cmd.epId, EpReg::TGT_COREID);
    Addr localAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr requestSize = dtu.regs().get(CmdReg::DATA_SIZE);
    Addr offset = dtu.regs().get(CmdReg::OFFSET);
    unsigned flags = dtu.regs().get(cmd.epId, EpReg::REQ_FLAGS);

    Addr targetAddr = dtu.regs().get(cmd.epId, EpReg::REQ_REM_ADDR);
    Addr remoteSize = dtu.regs().get(cmd.epId, EpReg::REQ_REM_SIZE);

    // we'll need that in writeComplete
    continueEvent.cmd = cmd;
    continueEvent.read = false;

    requestSize = std::min(dtu.maxNocPacketSize, requestSize);
    if(requestSize == 0)
        return;

    DPRINTFS(Dtu, (&dtu), "\e[1m[wr -> %u]\e[0m at offset %#018lx with EP%u from %#018lx:%lu\n",
        targetCoreId, offset, cmd.epId, localAddr, requestSize);

    // TODO error handling
    assert(flags & Dtu::MemoryFlags::WRITE);
    assert(requestSize + offset >= requestSize);
    assert(requestSize + offset <= remoteSize);

    dtu.startTransfer(Dtu::TransferType::LOCAL_READ,
                      NocAddr(targetCoreId, 0, targetAddr + offset),
                      localAddr,
                      requestSize);
}

void
MemoryUnit::readComplete(PacketPtr pkt)
{
    dtu.printPacket(pkt);

    Addr localAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr requestSize = dtu.regs().get(CmdReg::DATA_SIZE);
    Addr offset = dtu.regs().get(CmdReg::OFFSET);

    requestSize -= pkt->getSize();

    Cycles delay = dtu.ticksToCycles(pkt->headerDelay);

    dtu.startTransfer(Dtu::TransferType::LOCAL_WRITE,
                      NocAddr(0, 0),        // remote address is irrelevant
                      localAddr,
                      pkt->getSize(),
                      pkt,
                      NULL,
                      delay,
                      requestSize == 0);

    if(requestSize > 0)
    {
        dtu.regs().set(CmdReg::DATA_SIZE, requestSize);
        dtu.regs().set(CmdReg::DATA_ADDR, localAddr + pkt->getSize());
        dtu.regs().set(CmdReg::OFFSET, offset + pkt->getSize());

        // transfer the next packet
        dtu.schedule(continueEvent, dtu.clockEdge(Cycles(1)));
    }
}

void
MemoryUnit::writeComplete(PacketPtr pkt)
{
    Addr requestSize = dtu.regs().get(CmdReg::DATA_SIZE);

    // write finished or if requestSize < pkt->getSize(), it was a message
    if(requestSize <= pkt->getSize())
    {
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay + pkt->payloadDelay);
        dtu.scheduleFinishOp(delay);
    }
    // write needs to be continued
    else if(requestSize > pkt->getSize())
    {
        Addr localAddr = dtu.regs().get(CmdReg::DATA_ADDR);
        Addr offset = dtu.regs().get(CmdReg::OFFSET);

        dtu.regs().set(CmdReg::DATA_SIZE, requestSize - pkt->getSize());
        dtu.regs().set(CmdReg::DATA_ADDR, localAddr + pkt->getSize());
        dtu.regs().set(CmdReg::OFFSET, offset + pkt->getSize());

        // transfer the next packet
        dtu.schedule(continueEvent, dtu.clockEdge(Cycles(1)));
    }

    dtu.freeRequest(pkt);
}

void
MemoryUnit::recvFromNoc(PacketPtr pkt)
{
    DPRINTFS(Dtu, (&dtu), "\e[1m[%s <- ?]\e[0m %#018lx:%lu\n",
        pkt->isWrite() ? "wr" : "rd",
        NocAddr(pkt->getAddr()).offset,
        pkt->getSize());
    
    if(pkt->isWrite())
        dtu.printPacket(pkt);

    if (NocAddr(pkt->getAddr()).offset & dtu.regFileBaseAddr)
    {
        Addr oldAddr = pkt->getAddr();
        pkt->setAddr(NocAddr(oldAddr).offset);

        dtu.forwardRequestToRegFile(pkt, false);

        // as this is synchronous, we can restore the address right away
        pkt->setAddr(oldAddr);
    }
    else
    {
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;
        delay += dtu.nocRequestToSpmRequestLatency;

        auto type = pkt->isWrite() ? Dtu::TransferType::REMOTE_WRITE : Dtu::TransferType::REMOTE_READ;
        dtu.startTransfer(type,
                          NocAddr(0, 0),                    // remote address is irrelevant
                          NocAddr(pkt->getAddr()).offset,   // other remote is our local
                          pkt->getSize(),
                          pkt,
                          NULL,
                          delay);
    }
}
