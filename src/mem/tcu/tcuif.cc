/*
 * Copyright (C) 2017-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2021 Nils Asmussen, Barkhausen Institut
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

#include "mem/tcu/tcuif.hh"

TcuIf::TcuIf(Addr reg_base, RequestorID requestorId, unsigned int id)
    : reg_base(reg_base),
      requestorId(requestorId),
      id(id)
{
}

Addr
TcuIf::getRegAddr(PrivReg reg)
{
    return TcuTlb::PAGE_SIZE * 2 + static_cast<Addr>(reg) * sizeof(RegFile::reg_t);
}

Addr
TcuIf::getRegAddr(UnprivReg reg)
{
    Addr result = sizeof(RegFile::reg_t) * numExtRegs;

    result += static_cast<Addr>(reg) * sizeof(RegFile::reg_t);

    return result;
}

Addr
TcuIf::getRegAddr(unsigned reg, unsigned epid)
{
    Addr result = sizeof(RegFile::reg_t) * (numExtRegs + numUnprivRegs);

    result += epid * numEpRegs * sizeof(RegFile::reg_t);

    result += reg * sizeof(RegFile::reg_t);

    return result;
}

PacketPtr
TcuIf::createPacket(Addr paddr,
                    size_t size,
                    MemCmd cmd = MemCmd::WriteReq)
{
    return createPacket(paddr, new uint8_t[size], size, cmd);
}

PacketPtr
TcuIf::createPacket(Addr paddr,
                    const void *data,
                    size_t size,
                    MemCmd cmd = MemCmd::WriteReq)
{
    Request::Flags flags;

    auto req = std::make_shared<Request>(paddr, size, flags, requestorId);
    req->setContext(id);

    auto pkt = new Packet(req, cmd);
    pkt->dataDynamic(data);

    return pkt;
}

PacketPtr
TcuIf::createTcuRegPkt(Addr reg,
                       RegFile::reg_t value,
                       MemCmd cmd = MemCmd::WriteReq)
{
    auto pkt = createPacket(reg_base + reg, sizeof(RegFile::reg_t),
                            cmd);
    *pkt->getPtr<RegFile::reg_t>() = value;
    return pkt;
}

PacketPtr
TcuIf::createTcuCmdPkt(CmdCommand::Bits cmd,
                       CmdData::Bits data,
                       uint64_t offset)
{
    static_assert(static_cast<int>(UnprivReg::COMMAND) == 0, "");
    static_assert(static_cast<int>(UnprivReg::DATA) == 1, "");
    static_assert(static_cast<int>(UnprivReg::ARG1) == 2, "");

    auto pkt = createPacket(reg_base + getRegAddr(UnprivReg::COMMAND),
                            sizeof(RegFile::reg_t) * 3,
                            MemCmd::WriteReq);

    RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
    regs[0] = cmd;
    regs[1] = data;
    regs[2] = offset;
    return pkt;
}

void
TcuIf::freePacket(PacketPtr pkt)
{
    // the packet will delete the data
    delete pkt;
}
