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

#include "debug/Tcu.hh"
#include "debug/TcuBuf.hh"
#include "debug/TcuPackets.hh"
#include "mem/tcu/mem_unit.hh"
#include "mem/tcu/xfer_unit.hh"
#include "mem/tcu/noc_addr.hh"

static uint cmdToXferFlags(uint flags) {
    return flags & 1;
}

static uint cmdToNocFlags(uint flags) {
    return flags & 1;
}

static uint nocToXferFlags(uint flags) {
    return flags & 3;
}

static uint xferToNocFlags(uint flags) {
    return flags & 3;
}

static void
finishReadWrite(Tcu &tcu, Addr size)
{
    // change data register accordingly
    DataReg data = tcu.regs().getDataReg();
    data.size -= size;
    data.addr += size;
    tcu.regs().setDataReg(data);

    // change offset
    Addr offset = tcu.regs().get(CmdReg::ARG1);
    offset += size;
    tcu.regs().set(CmdReg::ARG1, offset);
}

void
MemoryUnit::regStats()
{
    readBytes
        .init(8)
        .name(tcu.name() + ".mem.readBytes")
        .desc("Sent read requests (in bytes)")
        .flags(Stats::nozero);
    writtenBytes
        .init(8)
        .name(tcu.name() + ".mem.writtenBytes")
        .desc("Sent write requests (in bytes)")
        .flags(Stats::nozero);
    receivedBytes
        .init(8)
        .name(tcu.name() + ".mem.receivedBytes")
        .desc("Received read/write requests (in bytes)")
        .flags(Stats::nozero);
    wrongVPE
        .name(tcu.name() + ".mem.wrongVPE")
        .desc("Number of received requests that targeted the wrong VPE")
        .flags(Stats::nozero);
}

void
MemoryUnit::startRead(const Tcu::Command::Bits& cmd)
{
    MemEp ep = tcu.regs().getMemEp(cmd.epid);

    if(ep.flags == 0 || ep.vpe != tcu.regs().getVPE())
    {
        tcu.scheduleFinishOp(Cycles(1), TcuError::INV_EP);
        return;
    }

    if(!(ep.flags & Tcu::MemoryFlags::READ) || (ep.vpe != tcu.regs().getVPE()))
    {
        tcu.scheduleFinishOp(Cycles(1), TcuError::NO_PERM);
        return;
    }

    DataReg data = tcu.regs().getDataReg();
    Addr offset = tcu.regs().get(CmdReg::ARG1);
    Addr size = std::min(static_cast<Addr>(data.size), tcu.maxNocPacketSize);

    readBytes.sample(size);

    DPRINTFS(Tcu, (&tcu),
        "\e[1m[rd -> %u]\e[0m at %#018lx+%#lx with EP%u into %#018lx:%lu\n",
        ep.targetPe, ep.remoteAddr, offset,
        cmd.epid, data.addr, size);

    if(size == 0)
    {
        tcu.scheduleFinishOp(Cycles(1), TcuError::NONE);
        return;
    }

    if(size + offset < size || size + offset > ep.remoteSize)
    {
        tcu.scheduleFinishOp(Cycles(1), TcuError::INV_ARGS);
        return;
    }

    NocAddr nocAddr(ep.targetPe, ep.remoteAddr + offset);

    uint flags = cmdToNocFlags(cmd.flags);

    if (tcu.coherent && !tcu.mmioRegion.contains(nocAddr.offset) &&
        tcu.isMemPE(nocAddr.peId))
    {
        flags |= XferUnit::NOXLATE;

        auto xfer = new LocalReadTransferEvent(nocAddr.getAddr(),
                                               data.addr,
                                               size,
                                               flags);
        tcu.startTransfer(xfer, Cycles(1));
    }
    else
    {
        auto pkt = tcu.generateRequest(nocAddr.getAddr(),
                                       size,
                                       MemCmd::ReadReq);

        tcu.sendNocRequest(Tcu::NocPacketType::READ_REQ,
                           pkt,
                           ep.targetVpe,
                           flags,
                           tcu.commandToNocRequestLatency);
    }
}

bool
MemoryUnit::LocalReadTransferEvent::transferDone(TcuError result)
{
    Cycles delay(1);

    if (result != TcuError::NONE)
    {
        tcu().scheduleFinishOp(delay, result);
        return true;
    }

    uint wflags = flags() & ~XferUnit::NOXLATE;
    uint8_t *tmp = new uint8_t[size()];
    memcpy(tmp, data(), size());

    auto xfer = new LocalWriteTransferEvent(dest, tmp, size(), wflags);
    tcu().startTransfer(xfer, delay);
    return true;
}

void
MemoryUnit::LocalWriteTransferEvent::transferStart()
{
    memcpy(data(), tmp, tmpSize);
    delete[] tmp;
}

bool
MemoryUnit::LocalWriteTransferEvent::transferDone(TcuError result)
{
    if (result == TcuError::NONE)
        finishReadWrite(tcu(), tmpSize);

    tcu().scheduleFinishOp(Cycles(1), result);
    return true;
}

void
MemoryUnit::readComplete(const Tcu::Command::Bits& cmd, PacketPtr pkt, TcuError error)
{
    tcu.printPacket(pkt);

    const DataReg data = tcu.regs().getDataReg();

    // since the transfer is done in steps, we can start after the header
    // delay here
    Cycles delay = tcu.ticksToCycles(pkt->headerDelay);

    if (error != TcuError::NONE)
    {
        tcu.scheduleFinishOp(delay, error);
        return;
    }

    uint flags = cmdToXferFlags(cmd.flags);
    auto xfer = new ReadTransferEvent(data.addr, flags, pkt);
    tcu.startTransfer(xfer, delay);
}

void
MemoryUnit::ReadTransferEvent::transferStart()
{
    // here is also no additional delay, because we are doing that in
    // parallel and are already paying for it at other places
    memcpy(data(), pkt->getPtr<uint8_t>(), pkt->getSize());
}

bool
MemoryUnit::ReadTransferEvent::transferDone(TcuError result)
{
    if (result == TcuError::NONE)
        finishReadWrite(tcu(), pkt->getSize());

    tcu().scheduleFinishOp(Cycles(1), result);

    tcu().freeRequest(pkt);
    return true;
}

void
MemoryUnit::startWrite(const Tcu::Command::Bits& cmd)
{
    MemEp ep = tcu.regs().getMemEp(cmd.epid);

    if(ep.flags == 0 || ep.vpe != tcu.regs().getVPE())
    {
        tcu.scheduleFinishOp(Cycles(1), TcuError::INV_EP);
        return;
    }

    if(!(ep.flags & Tcu::MemoryFlags::WRITE))
    {
        tcu.scheduleFinishOp(Cycles(1), TcuError::NO_PERM);
        return;
    }

    DataReg data = tcu.regs().getDataReg();
    Addr offset = tcu.regs().get(CmdReg::ARG1);
    Addr size = std::min(static_cast<Addr>(data.size), tcu.maxNocPacketSize);

    writtenBytes.sample(size);

    DPRINTFS(Tcu, (&tcu),
        "\e[1m[wr -> %u]\e[0m at %#018lx+%#lx with EP%u from %#018lx:%lu\n",
        ep.targetPe, ep.remoteAddr, offset,
        cmd.epid, data.addr, size);

    if(size == 0)
    {
        tcu.scheduleFinishOp(Cycles(1), TcuError::NONE);
        return;
    }

    if(size + offset < size || size + offset > ep.remoteSize)
    {
        tcu.scheduleFinishOp(Cycles(1), TcuError::INV_ARGS);
        return;
    }

    NocAddr dest(ep.targetPe, ep.remoteAddr + offset);

    uint flags = cmdToXferFlags(cmd.flags);
    auto xfer = new WriteTransferEvent(
        data.addr, size, ep.targetVpe, flags, dest);
    tcu.startTransfer(xfer, Cycles(0));
}

bool
MemoryUnit::WriteTransferEvent::transferDone(TcuError result)
{
    if (result != TcuError::NONE)
    {
        tcu().scheduleFinishOp(Cycles(1), result);
    }
    else
    {
        auto pkt = tcu().generateRequest(dest.getAddr(),
                                         size(),
                                         MemCmd::WriteReq);
        memcpy(pkt->getPtr<uint8_t>(), data(), size());

        Cycles delay = tcu().transferToNocLatency;
        tcu().printPacket(pkt);

        if (tcu().coherent && !tcu().mmioRegion.contains(dest.offset) &&
            tcu().isMemPE(dest.peId))
        {
            uint rflags = (flags() & XferUnit::NOPF) | XferUnit::NOXLATE;

            auto xfer = new ReadTransferEvent(dest.getAddr(), rflags, pkt);
            tcu().startTransfer(xfer, delay);
        }
        else
        {
            Tcu::NocPacketType pktType;
            if (flags() & XferUnit::MESSAGE)
                pktType = Tcu::NocPacketType::MESSAGE;
            else
                pktType = Tcu::NocPacketType::WRITE_REQ;

            uint rflags = xferToNocFlags(flags());
            tcu().setCommandSent();
            tcu().sendNocRequest(pktType, pkt, vpe, rflags, delay);
        }
    }
    return true;
}

void
MemoryUnit::writeComplete(const Tcu::Command::Bits& cmd, PacketPtr pkt, TcuError error)
{
    if (cmd.opcode == Tcu::Command::WRITE && error == TcuError::NONE)
        finishReadWrite(tcu, pkt->getSize());

    // we don't need to pay the payload delay here because the message
    // basically has no payload since we only receive an ACK back for
    // writing
    Cycles delay = tcu.ticksToCycles(pkt->headerDelay);
    tcu.scheduleFinishOp(delay, error);

    tcu.freeRequest(pkt);
}

void
MemoryUnit::recvFunctionalFromNoc(PacketPtr pkt)
{
    // set the local address
    pkt->setAddr(NocAddr(pkt->getAddr()).offset);

    tcu.sendFunctionalMemRequest(pkt);
}

TcuError
MemoryUnit::recvFromNoc(vpeid_t tvpe, PacketPtr pkt, uint flags)
{
    NocAddr addr(pkt->getAddr());

    DPRINTFS(Tcu, (&tcu), "\e[1m[%s <- ?]\e[0m %#018lx:%lu (VPE %u)\n",
        pkt->isWrite() ? "wr" : "rd",
        addr.offset,
        pkt->getSize(),
        tvpe);

    if (pkt->isWrite())
        tcu.printPacket(pkt);

    receivedBytes.sample(pkt->getSize());

    if (tcu.mmioRegion.contains(addr.offset))
    {
        pkt->setAddr(addr.offset);

        tcu.forwardRequestToRegFile(pkt, false);

        // as this is synchronous, we can restore the address right away
        pkt->setAddr(addr.getAddr());
    }
    else
    {
        // the same as above: the transfer happens piece by piece and we can
        // start after the header
        Cycles delay = tcu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;

        auto type = pkt->isWrite() ? XferUnit::TransferType::REMOTE_WRITE
                                   : XferUnit::TransferType::REMOTE_READ;
        uint xflags = nocToXferFlags(flags);

        auto *ev = new ReceiveTransferEvent(
            type, addr.offset, tvpe, xflags, pkt);
        tcu.startTransfer(ev, delay);
    }

    return TcuError::NONE;
}

void
MemoryUnit::ReceiveTransferEvent::transferStart()
{
    // the memory access refers to the VPE given by the NoC request, not
    // necessarily the currently running VPE
    vpeId(vpe);

    if (pkt->isWrite())
    {
        // here is also no additional delay, because we are doing that in
        // parallel and are already paying for it at other places
        memcpy(data(), pkt->getPtr<uint8_t>(), pkt->getSize());
    }
}

bool
MemoryUnit::ReceiveTransferEvent::transferDone(TcuError result)
{
    // some requests from the cache (e.g. cleanEvict) do not need a
    // response
    if (pkt->needsResponse())
    {
        pkt->makeResponse();

        if (pkt->isRead())
            memcpy(pkt->getPtr<uint8_t>(), data(), size());

        // set result
        auto state = dynamic_cast<Tcu::NocSenderState*>(pkt->senderState);
        state->result = result;

        Cycles delay = tcu().transferToNocLatency;
        tcu().schedNocResponse(pkt, tcu().clockEdge(delay));
    }
    return true;
}
