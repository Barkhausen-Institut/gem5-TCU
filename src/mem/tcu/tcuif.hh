/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
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

#ifndef __MEM_TCU_IF_HH__
#define __MEM_TCU_IF_HH__

#include "mem/tcu/tcu.hh"

class TcuIf {
public:
    static Addr getRegAddr(PrivReg reg);

    static Addr getRegAddr(UnprivReg reg);

    static Addr getRegAddr(unsigned reg, unsigned epid);

    explicit TcuIf(Addr reg_base, RequestorID requestorId, unsigned int id);

    RequestorID reqId() const { return requestorId; }

    PacketPtr createPacket(Addr paddr, size_t size, MemCmd cmd);

    PacketPtr createPacket(Addr paddr, const void *data, size_t size,
                           MemCmd cmd);

    PacketPtr createTcuRegPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    PacketPtr createTcuCmdPkt(CmdCommand::Bits cmd, CmdData::Bits data,
                              uint64_t offset = 0);

    void freePacket(PacketPtr pkt);

private:
    Addr reg_base;
    RequestorID requestorId;
    unsigned int id;
};

#endif
