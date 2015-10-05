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

    DPRINTFS(Dtu, (&dtu), "\e[1m[rd -> %u]\e[0m at offset %#018lx with EP%u into %#018lx:%lu\n",
        targetCoreId, offset, cmd.epId, localAddr, requestSize);

    // TODO error handling
    assert(flags & Dtu::MemoryFlags::READ);
    assert(requestSize > 0);
    assert(requestSize + offset >= requestSize);
    assert(requestSize + offset <= remoteSize);
    //assert(requestSize < maxNocPacketSize);

    dtu.startTransfer(Dtu::NocPacketType::READ_REQ,
                      NocAddr(targetCoreId, 0, remoteAddr + offset),
                      localAddr,
                      requestSize);
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

    DPRINTFS(Dtu, (&dtu), "\e[1m[wr -> %u]\e[0m at offset %#018lx with EP%u from %#018lx:%lu\n",
        targetCoreId, offset, cmd.epId, localAddr, requestSize);

    // TODO error handling
    assert(requestSize > 0);
    assert(flags & Dtu::MemoryFlags::WRITE);
    assert(requestSize == dtu.regs().get(CmdReg::DATA_SIZE));
    assert(requestSize + offset >= requestSize);
    assert(requestSize + offset <= remoteSize);
    //assert(requestSize <= maxNocPacketSize);

    dtu.startTransfer(Dtu::NocPacketType::WRITE_REQ,
                      NocAddr(targetCoreId, 0, targetAddr + offset),
                      localAddr,
                      requestSize);
}

void
MemoryUnit::sendWriteToNoc(const uint8_t* data,
                           Addr requestSize,
                           Tick spmPktHeaderDelay,
                           Tick spmPktPayloadDelay)
{
    Dtu::Command cmd = dtu.getCommand();
    unsigned epId = cmd.epId;

    unsigned targetCoreId = dtu.regs().get(epId, EpReg::TGT_COREID);
    Addr offset = dtu.regs().get(CmdReg::OFFSET);
    Addr targetAddr = dtu.regs().get(epId, EpReg::REQ_REM_ADDR);
    Addr remoteSize = dtu.regs().get(epId, EpReg::REQ_REM_SIZE);

    DPRINTFS(DtuDetail, (&dtu), "Send %lu bytes to address %#018lx in PE%u.\n",
                 requestSize,
                 targetAddr + offset,
                 targetCoreId);

    // TODO error handling
    assert(requestSize > 0);
    assert(requestSize == dtu.regs().get(CmdReg::DATA_SIZE));
    assert(requestSize + offset >= requestSize);
    assert(requestSize + offset <= remoteSize);
    //assert(requestSize <= maxNocPacketSize);

    auto pkt = dtu.generateRequest(NocAddr(targetCoreId, 0, targetAddr + offset).getAddr(),
                                   requestSize,
                                   MemCmd::WriteReq);

    memcpy(pkt->getPtr<uint8_t>(),
           data,
           requestSize);

    /*
     * See sendNocMessage() for an explanation of delay handling.
     */
    Cycles delay = dtu.spmResponseToNocRequestLatency;
    delay += dtu.ticksToCycles(spmPktHeaderDelay);
    pkt->payloadDelay = spmPktPayloadDelay;
    dtu.printPacket(pkt);
    dtu.sendNocRequest(Dtu::NocPacketType::WRITE_REQ, pkt, delay);
}

void
MemoryUnit::sendToSpmComplete(PacketPtr pkt, bool last)
{
    if(last)
    {
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay + pkt->payloadDelay);

        dtu.scheduleFinishOp(delay);
    }
}

void
MemoryUnit::readComplete(PacketPtr pkt)
{
    auto cmd = dtu.getCommand();

    Addr localAddr = dtu.regs().get(CmdReg::DATA_ADDR);
    Addr remoteAddr = dtu.regs().get(cmd.epId, EpReg::REQ_REM_ADDR);
    Addr reqOffset = dtu.regs().get(CmdReg::OFFSET);
    Addr offset = (NocAddr(pkt->getAddr()).offset - reqOffset) - remoteAddr;

    dtu.printPacket(pkt);

    DPRINTFS(DtuDetail, (&dtu), "Write %lu bytes to local scratchpad at address %#018lx.\n",
             pkt->getSize(),
             localAddr + offset);

    Cycles delay = dtu.ticksToCycles(pkt->headerDelay);

    auto spmPkt = dtu.generateRequest(localAddr + offset, pkt->getSize(), MemCmd::WriteReq);
    spmPkt->payloadDelay = pkt->payloadDelay;

    memcpy(spmPkt->getPtr<uint8_t>(), pkt->getPtr<uint8_t>(), pkt->getSize());

    dtu.sendSpmRequest(spmPkt,
                       cmd.epId,
                       delay + dtu.nocResponseToSpmRequestLatency,
                       Dtu::SpmPacketType::LOCAL_REQUEST,
                       NocAddr(pkt->getAddr()).last);
}

void
MemoryUnit::writeComplete(PacketPtr pkt)
{
    if(NocAddr(pkt->getAddr()).last)
    {
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay + pkt->payloadDelay);

        dtu.scheduleFinishOp(delay);
    }
}

void
MemoryUnit::recvFromNoc(PacketPtr pkt)
{
    Addr oldAddr = pkt->getAddr();

    // get local Address
    pkt->setAddr(NocAddr(oldAddr).offset);
    pkt->req->setPaddr(NocAddr(oldAddr).offset);

    DPRINTFS(Dtu, (&dtu), "\e[1m[%s <- ?]\e[0m %#018lx:%lu\n",
        pkt->isWrite() ? "wr" : "rd",
        pkt->getAddr(),
        pkt->getSize());
    
    if(pkt->isWrite())
        dtu.printPacket(pkt);

    if (pkt->getAddr() & dtu.regFileBaseAddr)
    {
        dtu.forwardRequestToRegFile(pkt, false);

        // as this is synchronous, we can restore the address right away
        pkt->setAddr(oldAddr);
        pkt->req->setPaddr(oldAddr);
    }
    else
    {
        Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;
        delay += dtu.nocRequestToSpmRequestLatency;

        // remember the old address for later
        auto state = new Dtu::AddrSenderState;
        state->addr = oldAddr;
        pkt->pushSenderState(state);

        dtu.sendSpmRequest(pkt,
                           -1,
                           delay,
                           Dtu::SpmPacketType::FORWARDED_REQUEST,
                           false);
    }
}

void
MemoryUnit::recvFromNocComplete(PacketPtr pkt)
{
    if (!dtu.atomicMode)
    {
        DPRINTFS(DtuDetail, (&dtu), "Forwarded request to Scratchpad. Send response back via NoC\n");

        Cycles delay = dtu.ticksToCycles(pkt->headerDelay + pkt->payloadDelay);
        delay += dtu.spmResponseToNocResponseLatency;

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;
        if(pkt->isRead())
            dtu.printPacket(pkt);

        // restore old address
        auto state = dynamic_cast<Dtu::AddrSenderState*>(pkt->popSenderState());
        pkt->setAddr(state->addr);
        pkt->req->setPaddr(state->addr);
        delete state;

        dtu.schedNocResponse(pkt, dtu.clockEdge(delay));
    }
}
