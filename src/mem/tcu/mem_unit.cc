/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
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
#include "mem/tcu/reg_file.hh"

namespace gem5
{
namespace tcu
{

static void
finishReadWrite(Tcu &tcu, Addr size)
{
    // change data register accordingly
    CmdData data = tcu.regs().getData();
    data.size = data.size - size;
    data.addr = data.addr + size;
    tcu.regs().setData(data);

    // change offset
    Addr offset = tcu.regs().get(UnprivReg::ARG1);
    offset += size;
    tcu.regs().set(UnprivReg::ARG1, offset);
}

void
MemoryUnit::regStats()
{
    readBytes
        .init(8)
        .name(tcu.name() + ".mem.readBytes")
        .desc("Sent read requests (in bytes)")
        .flags(statistics::nozero);
    writtenBytes
        .init(8)
        .name(tcu.name() + ".mem.writtenBytes")
        .desc("Sent write requests (in bytes)")
        .flags(statistics::nozero);
    receivedBytes
        .init(8)
        .name(tcu.name() + ".mem.receivedBytes")
        .desc("Received read/write requests (in bytes)")
        .flags(statistics::nozero);
    wrongAct
        .name(tcu.name() + ".mem.wrongAct")
        .desc("Number of received requests that targeted the wrong activity")
        .flags(statistics::nozero);
}

void
MemoryUnit::startRead(const CmdCommand::Bits& cmd)
{
    eps.addEp(cmd.epid);
    eps.onFetched(
        std::bind(&MemoryUnit::startReadWithEP, this, std::placeholders::_1));
}

void
MemoryUnit::startReadWithEP(EpFile::EpCache &eps)
{
    Cycles delay(tcu.cmdReadLatency);
    CmdCommand::Bits cmd = tcu.regs().getCommand();

    const Ep ep = eps.getEp(cmd.epid);

    if(ep.type() != EpType::MEMORY)
    {
        return tcu.schedCmdError(delay, TcuError::NO_MEP,
                                 "EP%u: invalid EP\n", cmd.epid);
    }

    const MemEp mep = ep.mem;

    if(mep.r0.act != tcu.regs().getCurAct().id)
    {
        return tcu.schedCmdError(delay, TcuError::FOREIGN_EP,
                                 "EP%u: foreign EP\n", cmd.epid);
    }

    if(!(mep.r0.flags & Tcu::MemoryFlags::READ))
    {
        return tcu.schedCmdError(delay, TcuError::NO_PERM,
                                 "EP%u: no permission\n", cmd.epid);
    }

    CmdData data = tcu.regs().getData();
    Addr offset = tcu.regs().get(UnprivReg::ARG1);
    Addr size = std::min(static_cast<Addr>(data.size), tcu.maxNocPacketSize);

    if(size == 0)
    {
        tcu.scheduleCmdFinish(delay, TcuError::NONE);
        return;
    }

    if(size + offset < size || size + offset > mep.r2.remoteSize)
    {
        return tcu.schedCmdError(delay, TcuError::OUT_OF_BOUNDS,
                                 "EP%u: out of bounds\n", cmd.epid);
    }

    auto data_page = data.addr & ~static_cast<Addr>(TcuTlb::PAGE_MASK);
    if(data_page !=
       ((data.addr + size - 1) & ~static_cast<Addr>(TcuTlb::PAGE_MASK)))
    {
        return tcu.schedCmdError(delay, TcuError::PAGE_BOUNDARY,
                                 "EP%u: data contains page boundary\n", cmd.epid);
    }

    readBytes.sample(size);

    NocAddr dest(TileId::from_raw(mep.r0.targetTile),
                 mep.r1.remoteAddr + offset);

    DPRINTFS(Tcu, (&tcu),
        "\e[1m[rd -> %s]\e[0m at %#018lx+%#lx with EP%u into %#018lx:%lu\n",
        dest.tileId, mep.r1.remoteAddr, offset,
        cmd.epid, data.addr, size);

    auto pkt = tcu.generateRequest(dest.getAddr(),
                                   size,
                                   MemCmd::ReadReq);

    tcu.sendNocRequest(Tcu::NocPacketType::READ_REQ,
                       pkt,
                       delay);
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

    tcu().scheduleCmdFinish(Cycles(1), result);
    return true;
}

void
MemoryUnit::readComplete(const CmdCommand::Bits& cmd, PacketPtr pkt, TcuError error)
{
    tcu.printPacket(pkt);

    const CmdData data = tcu.regs().getData();

    // since the transfer is done in steps, we can start after the header
    // delay here
    Cycles delay = tcu.ticksToCycles(pkt->headerDelay);

    if (error != TcuError::NONE)
    {
        tcu.scheduleCmdFinish(delay, error);
        return;
    }

    NocAddr phys(data.addr);
    if (tcu.tlb())
    {
        Cycles tlbLatency;
        auto asid = tcu.regs().getCurAct().id;
        auto res = tcu.tlb()->lookup(data.addr, asid, TcuTlb::WRITE,
                                     &phys, &tlbLatency);
        delay += tlbLatency;

        if (res != TcuTlb::HIT)
        {
            return tcu.schedCmdError(Cycles(1), TcuError::TRANSLATION_FAULT,
                                     "EP%u: TLB miss for data address\n",
                                     cmd.epid);
        }
    }

    auto xfer = new ReadTransferEvent(phys, 0, pkt);
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

    tcu().scheduleCmdFinish(Cycles(1), result);

    tcu().freeRequest(pkt);
    return true;
}

void
MemoryUnit::startWrite(const CmdCommand::Bits& cmd)
{
    eps.addEp(cmd.epid);
    eps.onFetched(
        std::bind(&MemoryUnit::startWriteWithEP, this, std::placeholders::_1));
}

void
MemoryUnit::startWriteWithEP(EpFile::EpCache &eps)
{
    Cycles delay(tcu.cmdWriteLatency);
    CmdCommand::Bits cmd = tcu.regs().getCommand();

    const Ep ep = eps.getEp(cmd.epid);

    if(ep.type() != EpType::MEMORY)
    {
        return tcu.schedCmdError(delay, TcuError::NO_MEP,
                                 "EP%u: invalid EP\n", cmd.epid);
    }

    const MemEp mep = ep.mem;

    if(mep.r0.act != tcu.regs().getCurAct().id)
    {
        return tcu.schedCmdError(delay, TcuError::FOREIGN_EP,
                                 "EP%u: foreign EP\n", cmd.epid);
    }

    if(!(mep.r0.flags & Tcu::MemoryFlags::WRITE))
    {
        return tcu.schedCmdError(delay, TcuError::NO_PERM,
                                 "EP%u: no permission\n", cmd.epid);
    }

    CmdData data = tcu.regs().getData();
    Addr offset = tcu.regs().get(UnprivReg::ARG1);
    Addr size = std::min(static_cast<Addr>(data.size), tcu.maxNocPacketSize);

    if(size == 0)
    {
        tcu.scheduleCmdFinish(delay, TcuError::NONE);
        return;
    }

    if(size + offset < size || size + offset > mep.r2.remoteSize)
    {
        return tcu.schedCmdError(delay, TcuError::OUT_OF_BOUNDS,
                                 "EP%u: out of bounds\n", cmd.epid);
    }

    auto data_page = data.addr & ~static_cast<Addr>(TcuTlb::PAGE_MASK);
    if(data_page !=
        ((data.addr + size - 1) & ~static_cast<Addr>(TcuTlb::PAGE_MASK)))
    {
        return tcu.schedCmdError(delay, TcuError::PAGE_BOUNDARY,
                                 "EP%u: data contains page boundary\n",
                                 cmd.epid);
    }

    NocAddr phys(data.addr);
    if (tcu.tlb())
    {
        Cycles tlbLatency;
        auto asid = tcu.regs().getCurAct().id;
        auto res = tcu.tlb()->lookup(data.addr, asid, TcuTlb::READ,
                                     &phys, &tlbLatency);
        delay += tlbLatency;

        if (res != TcuTlb::HIT)
        {
            return tcu.schedCmdError(delay, TcuError::TRANSLATION_FAULT,
                                     "EP%u: TLB miss for data address\n",
                                     cmd.epid);
        }
    }

    NocAddr dest(TileId::from_raw(mep.r0.targetTile),
                 mep.r1.remoteAddr + offset);

    auto xfer = new WriteTransferEvent(phys, size, 0, dest);
    tcu.startTransfer(xfer, delay);
}

bool
MemoryUnit::WriteTransferEvent::transferDone(TcuError result)
{
    if (result != TcuError::NONE)
    {
        tcu().scheduleCmdFinish(Cycles(1), result);
    }
    else
    {
        auto pkt = tcu().generateRequest(dest.getAddr(),
                                         size(),
                                         MemCmd::WriteReq);
        memcpy(pkt->getPtr<uint8_t>(), data(), size());

        tcu().printPacket(pkt);

        Tcu::NocPacketType pktType;
        if (flags() & XferUnit::MESSAGE)
            pktType = Tcu::NocPacketType::MESSAGE;
        else
            pktType = Tcu::NocPacketType::WRITE_REQ;

        const CmdCommand::Bits cmd = tcu().regs().getCommand();
        if (cmd.opcode == CmdCommand::WRITE)
        {
            const CmdData data = tcu().regs().getData();
            Addr offset = tcu().regs().get(UnprivReg::ARG1);
            DPRINTFS(Tcu, (&tcu()),
                "\e[1m[wr -> %s]\e[0m at %#018lx+%#lx with EP%u from %#018lx:%lu\n",
                dest.tileId, dest.offset - offset, offset,
                cmd.epid, data.addr, size());
        }

        tcu().sendNocRequest(pktType, pkt, Cycles(1));
    }

    return true;
}

void
MemoryUnit::writeComplete(const CmdCommand::Bits& cmd, PacketPtr pkt, TcuError error)
{
    if (cmd.opcode == CmdCommand::WRITE && error == TcuError::NONE)
    {
        writtenBytes.sample(pkt->getSize());
        finishReadWrite(tcu, pkt->getSize());
    }

    // we don't need to pay the payload delay here because the message
    // basically has no payload since we only receive an ACK back for
    // writing
    Cycles delay = tcu.ticksToCycles(pkt->headerDelay);
    tcu.scheduleCmdFinish(delay, error);

    tcu.freeRequest(pkt);
}

void
MemoryUnit::recvFunctionalFromNoc(PacketPtr pkt)
{
    // set the local address
    pkt->setAddr(NocAddr(pkt->getAddr()).offset);

    tcu.sendFunctionalMemRequest(pkt);
}

void
MemoryUnit::recvFromNoc(PacketPtr pkt)
{
    NocAddr addr(pkt->getAddr());

    DPRINTFS(Tcu, (&tcu), "\e[1m[%s <- ?]\e[0m %#018lx:%lu\n",
        pkt->isWrite() ? "wr" : "rd",
        addr.offset,
        pkt->getSize());

    if (pkt->isWrite())
        tcu.printPacket(pkt);

    receivedBytes.sample(pkt->getSize());

    if (tcu.mmioRegion.contains(addr.offset))
        tcu.forwardRequestToRegFile(pkt, false);
    else
    {
        // the same as above: the transfer happens piece by piece and we can
        // start after the header
        Cycles delay = tcu.ticksToCycles(pkt->headerDelay);
        pkt->headerDelay = 0;

        auto type = pkt->isWrite() ? XferUnit::TransferType::REMOTE_WRITE
                                   : XferUnit::TransferType::REMOTE_READ;
        // accesses from remote TCUs always refer to physical memory
        auto *ev = new ReceiveTransferEvent(type, NocAddr(addr.offset), 0, pkt);
        tcu.startTransfer(ev, delay);
    }
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

        tcu().schedNocResponse(pkt, tcu().clockEdge(Cycles(1)));
    }
    return true;
}

}
}
