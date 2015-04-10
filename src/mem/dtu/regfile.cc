/*
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

#include "base/trace.hh"
#include "debug/DtuReg.hh"
#include "mem/dtu/regfile.hh"

RegFile::RegFile(const std::string name)
    : regFile(numRegs),
      _name(name)
{}

DtuReg
RegFile::readReg(DtuRegister reg) const
{
    DtuReg value = regFile[static_cast<Addr>(reg)];

    DPRINTF(DtuReg, "Read register %u (data 0x%x)\n",
                    static_cast<Addr>(reg),
                    value);

    return value;
}

void
RegFile::setReg(DtuRegister reg, DtuReg value)
{
    DPRINTF(DtuReg, "Set register %u (data 0x%x)\n",
                    static_cast<Addr>(reg),
                    value);

    regFile[static_cast<Addr>(reg)] = value;
}

bool
RegFile::isRegisterAddr(Addr addr) const
{
    // can't write in the middle of a register
    if (addr % sizeof(DtuReg) != 0)
        return false;

    if (addr / sizeof(DtuReg) >= numRegs)
        return false;

    return true;
}

Tick
RegFile::handleRequest(PacketPtr pkt)
{
    Addr paddr = pkt->getAddr();

    // we rely on the caller to test if this packet can be handled by the RegFile
    assert(isRegisterAddr(paddr));

    // we can only perform full register accesses
    assert(pkt->getSize() == sizeof(DtuReg));

    auto reg = static_cast<DtuRegister>(paddr / sizeof(DtuReg));
    DtuReg* data = pkt->getPtr<DtuReg>();

    if (pkt->isRead())
        *data = readReg(reg);
    else if (pkt->isWrite())
        setReg(reg, *data);
    else
        panic("unsopported packet type");

    if (pkt->needsResponse())
        pkt->makeResponse();

    // TODO latency calculation
    return 0;
}
