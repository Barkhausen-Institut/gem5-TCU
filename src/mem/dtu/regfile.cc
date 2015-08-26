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
#include "debug/DtuRegRange.hh"
#include "mem/dtu/regfile.hh"

const char *RegFile::dtuRegNames[] = {
    "COMMAND",
    "STATUS",
};

const char *RegFile::epRegNames[] = {
    "MODE",
    "MAX_MSG_SIZE",
    "BUF_MSG_CNT",
    "BUF_ADDR",
    "BUF_SIZE",
    "BUF_RD_PTR",
    "BUF_WR_PTR",
    "TGT_COREID",
    "TGT_EPID",
    "MSG_ADDR",
    "MSG_SIZE",
    "LABEL",
    "REPLY_EPID",
    "REPLY_LABEL",
    "REQ_LOC_ADDR",
    "REQ_REM_ADDR",
    "REQ_SIZE",
    "CREDITS",
};

Addr
RegFile::getRegAddr(DtuReg reg)
{
    return static_cast<Addr>(reg) * sizeof(reg_t);
}

Addr
RegFile::getRegAddr(EpReg reg, unsigned epid)
{
    Addr result = sizeof(reg_t) * numDtuRegs;

    result += epid * numEpRegs * sizeof(reg_t);

    result += static_cast<Addr>(reg) * sizeof(reg_t);

    return result;
}

RegFile::RegFile(const std::string name, unsigned _numEndpoints)
    : dtuRegs(numDtuRegs, 0),
      epRegs(_numEndpoints),
      numEndpoints(_numEndpoints),
      _name(name)
{
    for (int epid = 0; epid < numEndpoints; epid++)
    {
        for (int i = 0; i < numEpRegs; i++)
            epRegs[epid].push_back(0);
    }
}

RegFile::reg_t
RegFile::readDtuReg(DtuReg reg) const
{
    reg_t value = dtuRegs[static_cast<Addr>(reg)];

    DPRINTF(DtuReg, "DTU[%-12s] -> %#018x\n",
                    dtuRegNames[static_cast<Addr>(reg)],
                    value);

    return value;
}

void
RegFile::setDtuReg(DtuReg reg, reg_t value)
{
    DPRINTF(DtuReg, "DTU[%-12s] <- %#018x\n",
                    dtuRegNames[static_cast<Addr>(reg)],
                    value);

    dtuRegs[static_cast<Addr>(reg)] = value;
}

RegFile::reg_t
RegFile::readEpReg(unsigned epid, EpReg reg) const
{
    reg_t value = epRegs[epid][static_cast<Addr>(reg)];

    DPRINTF(DtuReg, "EP%u[%-12s] -> %#018x\n",
                    epid,
                    epRegNames[static_cast<Addr>(reg)],
                    value);

    return value;
}

void
RegFile::setEpReg(unsigned epid, EpReg reg, reg_t value)
{
    DPRINTF(DtuReg, "EP%u[%-12s] <- %#018x\n",
                    epid,
                    epRegNames[static_cast<Addr>(reg)],
                    value);

    epRegs[epid][static_cast<Addr>(reg)] = value;
}

bool
RegFile::handleRequest(PacketPtr pkt)
{
    assert(pkt->isRead() || pkt->isWrite());

    Addr pktAddr = pkt->getAddr();

    DPRINTF(DtuRegRange, "access @%#x, size=%u\n", pktAddr, pkt->getSize());

    // we can only perform full register accesses
    // TODO send error response instead of aborting
    assert(pkt->getSize() % sizeof(reg_t) == 0);
    assert(pktAddr % sizeof(reg_t) == 0);
    assert(pktAddr + pkt->getSize() <= getSize());

    reg_t* data = pkt->getPtr<reg_t>();

    // perform a single register access for each requested register
    for (unsigned offset = 0; offset < pkt->getSize(); offset += sizeof(reg_t))
    {
        Addr regAddr = pktAddr + offset;
        bool isEndpointAccess = regAddr >= sizeof(reg_t) * numDtuRegs;

        if (isEndpointAccess)
        {
            unsigned epid = (regAddr - sizeof(reg_t) * numDtuRegs) /
                            (sizeof(reg_t) * numEpRegs);

            unsigned regNumber = (regAddr / sizeof(reg_t) - numDtuRegs) % numEpRegs;

            auto reg = static_cast<EpReg>(regNumber);

            if (pkt->isRead())
                data[offset % sizeof(reg_t)] = readEpReg(epid, reg);
            else
                setEpReg(epid, reg, data[offset % sizeof(reg_t)]);
        }
        else
        {
            auto reg = static_cast<DtuReg>(pktAddr / sizeof(reg_t));

            if (pkt->isRead())
                data[offset % sizeof(reg_t)] = readDtuReg(reg);
            else if (pkt->isWrite())
                setDtuReg(reg, data[offset % sizeof(reg_t)]);
        }
    }

    if (pkt->needsResponse())
        pkt->makeResponse();

    if (pkt->isWrite() && pkt->getAddr() == static_cast<Addr>(DtuReg::COMMAND))
        return true;
    else
        return false;
}

Addr
RegFile::getSize() const
{
    Addr size = sizeof(reg_t) * numDtuRegs;

    size += sizeof(reg_t) * (numEndpoints * numEpRegs);

    return size;
}
