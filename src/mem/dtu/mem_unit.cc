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
finishReadWrite(Dtu &dtu, Addr size)
{
    // change data register accordingly
    DataReg data = dtu.regs().getDataReg();
    data.size -= size;
    data.addr += size;
    dtu.regs().setDataReg(data);

    // change command register
    Dtu::Command::Bits cmd = dtu.regs().get(CmdReg::COMMAND);
    cmd.arg = cmd.arg + size;
    dtu.regs().set(CmdReg::COMMAND, cmd);
}

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
MemoryUnit::startRead(const Dtu::Command::Bits& cmd)
{
    MemEp ep = dtu.regs().getMemEp(cmd.epid);

    if(ep.flags == 0 || ep.vpe != dtu.regs().get(PrivReg::VPE_ID))
    {
        dtu.scheduleFinishOp(Cycles(1), Dtu::Error::INV_EP);
        return;
    }

    if(!(ep.flags & Dtu::MemoryFlags::READ) || (ep.vpe != dtu.regs().get(PrivReg::VPE_ID)))
    {
        dtu.scheduleFinishOp(Cycles(1), Dtu::Error::NO_PERM);
        return;
    }

    DataReg data = dtu.regs().getDataReg();
    Addr offset = cmd.arg;
    Addr size = std::min(static_cast<Addr>(data.size), dtu.maxNocPacketSize);

    readBytes.sample(size);

    DPRINTFS(Dtu, (&dtu),
        "\e[1m[rd -> %u]\e[0m at %#018lx+%#lx with EP%u into %#018lx:%lu\n",
        ep.targetCore, ep.remoteAddr, offset,
        cmd.epid, data.addr, size);

    if(size == 0)
    {
        dtu.scheduleFinishOp(Cycles(1), Dtu::Error::NONE);
        return;
    }

    if(size + offset < size || size + offset > ep.remoteSize)
    {
        dtu.scheduleFinishOp(Cycles(1), Dtu::Error::INV_ARGS);
        return;
    }

    NocAddr nocAddr(ep.targetCore, ep.remoteAddr + offset);

    uint flags = cmdToNocFlags(cmd.flags);

    if (dtu.coherent && !dtu.mmioRegion.contains(nocAddr.offset) &&
        dtu.isMemPE(nocAddr.coreId))
    {
        flags |= XferUnit::NOXLATE;

        auto xfer = new LocalReadTransferEvent(nocAddr.getAddr(),
                                               data.addr,
                                               size,
                                               flags);
        dtu.startTransfer(xfer, Cycles(1));
    }
    else
    {
        auto pkt = dtu.generateRequest(nocAddr.getAddr(),
                                       size,
                                       MemCmd::ReadReq);

        dtu.sendNocRequest(Dtu::NocPacketType::READ_REQ,
                           pkt,
                           flags,
                           dtu.commandToNocRequestLatency);
    }
}

void
MemoryUnit::LocalReadTransferEvent::transferDone(Dtu::Error result)
{
    Cycles delay(1);

    if (result != Dtu::Error::NONE)
    {
        dtu().scheduleFinishOp(delay, result);
        return;
    }

    uint wflags = flags() & ~XferUnit::NOXLATE;
    uint8_t *tmp = new uint8_t[size()];
    memcpy(tmp, data(), size());

    auto xfer = new LocalWriteTransferEvent(dest, tmp, size(), wflags);
    dtu().startTransfer(xfer, delay);
}

void
MemoryUnit::LocalWriteTransferEvent::transferStart()
{
    memcpy(data(), tmp, tmpSize);
    delete[] tmp;
}

void
MemoryUnit::LocalWriteTransferEvent::transferDone(Dtu::Error result)
{
    if (result == Dtu::Error::NONE)
        finishReadWrite(dtu(), tmpSize);

    dtu().scheduleFinishOp(Cycles(1), result);
}

void
MemoryUnit::readComplete(const Dtu::Command::Bits& cmd, PacketPtr pkt, Dtu::Error error)
{
    dtu.printPacket(pkt);

    const DataReg data = dtu.regs().getDataReg();

    // since the transfer is done in steps, we can start after the header
    // delay here
    Cycles delay = dtu.ticksToCycles(pkt->headerDelay);

    if (error != Dtu::Error::NONE)
    {
        dtu.scheduleFinishOp(delay, error);
        return;
    }

    uint flags = cmdToXferFlags(cmd.flags);
    auto xfer = new ReadTransferEvent(data.addr, flags, pkt);
    dtu.startTransfer(xfer, delay);
}

void
MemoryUnit::ReadTransferEvent::transferStart()
{
    // here is also no additional delay, because we are doing that in
    // parallel and are already paying for it at other places
    memcpy(data(), pkt->getPtr<uint8_t>(), pkt->getSize());
}

void
MemoryUnit::ReadTransferEvent::transferDone(Dtu::Error result)
{
    if (result == Dtu::Error::NONE)
        finishReadWrite(dtu(), pkt->getSize());

    dtu().scheduleFinishOp(Cycles(1), result);

    dtu().freeRequest(pkt);
}

void
MemoryUnit::startWrite(const Dtu::Command::Bits& cmd)
{
    MemEp ep = dtu.regs().getMemEp(cmd.epid);

    if(ep.flags == 0 || ep.vpe != dtu.regs().get(PrivReg::VPE_ID))
    {
        dtu.scheduleFinishOp(Cycles(1), Dtu::Error::INV_EP);
        return;
    }

    if(!(ep.flags & Dtu::MemoryFlags::WRITE))
    {
        dtu.scheduleFinishOp(Cycles(1), Dtu::Error::NO_PERM);
        return;
    }

    DataReg data = dtu.regs().getDataReg();
    Addr offset = cmd.arg;
    Addr size = std::min(static_cast<Addr>(data.size), dtu.maxNocPacketSize);

    writtenBytes.sample(size);

    DPRINTFS(Dtu, (&dtu),
        "\e[1m[wr -> %u]\e[0m at %#018lx+%#lx with EP%u from %#018lx:%lu\n",
        ep.targetCore, ep.remoteAddr, offset,
        cmd.epid, data.addr, size);

    if(size == 0)
    {
        dtu.scheduleFinishOp(Cycles(1), Dtu::Error::NONE);
        return;
    }

    if(size + offset < size || size + offset > ep.remoteSize)
    {
        dtu.scheduleFinishOp(Cycles(1), Dtu::Error::INV_ARGS);
        return;
    }

    NocAddr dest(ep.targetCore, ep.remoteAddr + offset);

    uint flags = cmdToXferFlags(cmd.flags);
    auto xfer = new WriteTransferEvent(
        data.addr, size, flags, dest);
    dtu.startTransfer(xfer, Cycles(0));
}

void
MemoryUnit::WriteTransferEvent::transferDone(Dtu::Error result)
{
    if (result != Dtu::Error::NONE)
    {
        dtu().scheduleFinishOp(Cycles(1), result);
    }
    else
    {
        auto pkt = dtu().generateRequest(dest.getAddr(),
                                         size(),
                                         MemCmd::WriteReq);
        memcpy(pkt->getPtr<uint8_t>(), data(), size());

        Cycles delay = dtu().transferToNocLatency;
        dtu().printPacket(pkt);

        if (dtu().coherent && !dtu().mmioRegion.contains(dest.offset) &&
            dtu().isMemPE(dest.coreId))
        {
            uint rflags = (flags() & XferUnit::NOPF) | XferUnit::NOXLATE;

            auto xfer = new ReadTransferEvent(dest.getAddr(), rflags, pkt);
            dtu().startTransfer(xfer, delay);
        }
        else
        {
            Dtu::NocPacketType pktType;
            if (flags() & XferUnit::MESSAGE)
                pktType = Dtu::NocPacketType::MESSAGE;
            else
                pktType = Dtu::NocPacketType::WRITE_REQ;

            uint rflags = xferToNocFlags(flags());
            dtu().setCommandSent();
            dtu().sendNocRequest(pktType, pkt, rflags, delay);
        }
    }
}

void
MemoryUnit::writeComplete(const Dtu::Command::Bits& cmd, PacketPtr pkt, Dtu::Error error)
{
    if (cmd.opcode == Dtu::Command::WRITE && error == Dtu::Error::NONE)
        finishReadWrite(dtu, pkt->getSize());

    // we don't need to pay the payload delay here because the message
    // basically has no payload since we only receive an ACK back for
    // writing
    Cycles delay = dtu.ticksToCycles(pkt->headerDelay);
    dtu.scheduleFinishOp(delay, error);

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
MemoryUnit::recvFromNoc(PacketPtr pkt, uint flags)
{
    NocAddr addr(pkt->getAddr());

    DPRINTFS(Dtu, (&dtu), "\e[1m[%s <- ?]\e[0m %#018lx:%lu\n",
        pkt->isWrite() ? "wr" : "rd",
        addr.offset,
        pkt->getSize());

    if (pkt->isWrite())
        dtu.printPacket(pkt);

    receivedBytes.sample(pkt->getSize());

    if (dtu.mmioRegion.contains(addr.offset))
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
        uint xflags = nocToXferFlags(flags);

        auto *ev = new ReceiveTransferEvent(type, addr.offset, xflags, pkt);
        dtu.startTransfer(ev, delay);
    }

    return Dtu::Error::NONE;
}

void
MemoryUnit::ReceiveTransferEvent::transferStart()
{
    if (pkt->isWrite())
    {
        // here is also no additional delay, because we are doing that in
        // parallel and are already paying for it at other places
        memcpy(data(), pkt->getPtr<uint8_t>(), pkt->getSize());
    }
}

void
MemoryUnit::ReceiveTransferEvent::transferDone(Dtu::Error result)
{
    // some requests from the cache (e.g. cleanEvict) do not need a
    // response
    if (pkt->needsResponse())
    {
        pkt->makeResponse();

        if (pkt->isRead())
            memcpy(pkt->getPtr<uint8_t>(), data(), size());

        // set result
        auto state = dynamic_cast<Dtu::NocSenderState*>(pkt->senderState);
        state->result = result;

        Cycles delay = dtu().transferToNocLatency;
        dtu().schedNocResponse(pkt, dtu().clockEdge(delay));
    }
}
