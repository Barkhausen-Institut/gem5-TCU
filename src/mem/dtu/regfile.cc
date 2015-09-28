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

#include "base/trace.hh"
#include "debug/DtuReg.hh"
#include "debug/DtuRegRange.hh"
#include "mem/dtu/regfile.hh"

const char *RegFile::dtuRegNames[] = {
    "STATUS",
    "MSG_CNT",
};

const char *RegFile::cmdRegNames[] = {
    "COMMAND",
    "DATA_ADDR",
    "DATA_SIZE",
    "OFFSET",
    "REPLY_EPID",
    "REPLY_LABEL",
};

const char *RegFile::epRegNames[] = {
    "BUF_ADDR",
    "BUF_MSG_SIZE",
    "BUF_SIZE",
    "BUF_MSG_CNT",
    "BUF_RD_PTR",
    "BUF_WR_PTR",
    "TGT_COREID",
    "TGT_EPID",
    "MAX_MSG_SIZE",
    "LABEL",
    "CREDITS",
    "REQ_REM_ADDR",
    "REQ_REM_SIZE",
    "REQ_FLAGS",
};

RegFile::RegFile(const std::string& name, unsigned _numEndpoints)
    : dtuRegs(numDtuRegs, 0),
      cmdRegs(numCmdRegs, 0),
      epRegs(_numEndpoints),
      numEndpoints(_numEndpoints),
      _name(name)
{
    // at boot, all PEs are privileged
    set(DtuReg::STATUS, static_cast<reg_t>(Status::PRIV));

    for (int epid = 0; epid < numEndpoints; epid++)
    {
        for (int i = 0; i < numEpRegs; i++)
            epRegs[epid].push_back(0);
    }
}

RegFile::reg_t
RegFile::get(DtuReg reg) const
{
    reg_t value = dtuRegs[static_cast<Addr>(reg)];

    DPRINTF(DtuReg, "DTU[%-12s] -> %#018x\n",
                    dtuRegNames[static_cast<Addr>(reg)],
                    value);

    return value;
}

RegFile::reg_t
RegFile::get(CmdReg reg) const
{
    reg_t value = cmdRegs[static_cast<Addr>(reg)];

    DPRINTF(DtuReg, "CMD[%-12s] -> %#018x\n",
                    cmdRegNames[static_cast<Addr>(reg)],
                    value);

    return value;
}

RegFile::reg_t
RegFile::get(unsigned epid, EpReg reg) const
{
    reg_t value = epRegs[epid][static_cast<Addr>(reg)];

    DPRINTF(DtuReg, "EP%u[%-12s] -> %#018x\n",
                    epid,
                    epRegNames[static_cast<Addr>(reg)],
                    value);

    return value;
}

void
RegFile::set(DtuReg reg, reg_t value)
{
    DPRINTF(DtuReg, "DTU[%-12s] <- %#018x\n",
                    dtuRegNames[static_cast<Addr>(reg)],
                    value);

    dtuRegs[static_cast<Addr>(reg)] = value;
}

void
RegFile::set(CmdReg reg, reg_t value)
{
    DPRINTF(DtuReg, "CMD[%-12s] <- %#018x\n",
                    cmdRegNames[static_cast<Addr>(reg)],
                    value);

    cmdRegs[static_cast<Addr>(reg)] = value;
}

void
RegFile::set(unsigned epid, EpReg reg, reg_t value)
{
    DPRINTF(DtuReg, "EP%u[%-12s] <- %#018x\n",
                    epid,
                    epRegNames[static_cast<Addr>(reg)],
                    value);

    // update global message count
    if(reg == EpReg::BUF_MSG_CNT)
    {
        reg_t diff = value - epRegs[epid][static_cast<Addr>(reg)];
        reg_t old = dtuRegs[static_cast<Addr>(DtuReg::MSG_CNT)];
        set(DtuReg::MSG_CNT, old + diff);
    }

    epRegs[epid][static_cast<Addr>(reg)] = value;
}

bool
RegFile::handleRequest(PacketPtr pkt, bool isCpuRequest)
{
    assert(pkt->isRead() || pkt->isWrite());

    Addr pktAddr = pkt->getAddr();

    DPRINTF(DtuRegRange, "access @%#x, size=%u\n", pktAddr, pkt->getSize());

    // ignore invalid accesses (might happen due to speculative execution)
    // TODO maybe we can allow some of them later
    if((pkt->getSize() % sizeof(reg_t)) != 0 || (pktAddr % sizeof(reg_t)) != 0 ||
        pktAddr + pkt->getSize() > getSize()) {
        if (pkt->needsResponse())
            pkt->makeResponse();

        return false;
    }

    // we can only perform full register accesses
    assert(pkt->getSize() % sizeof(reg_t) == 0);
    assert(pktAddr % sizeof(reg_t) == 0);
    assert(pktAddr + pkt->getSize() <= getSize());

    reg_t* data = pkt->getPtr<reg_t>();
    bool cmdChanged = false;
    reg_t privFlag = static_cast<reg_t>(Status::PRIV);
    bool isPrivileged = get(DtuReg::STATUS) & privFlag;

    // perform a single register access for each requested register
    for (unsigned offset = 0; offset < pkt->getSize(); offset += sizeof(reg_t))
    {
        Addr regAddr = pktAddr + offset;

        // dtu register
        if(regAddr < sizeof(reg_t) * numDtuRegs)
        {
            auto reg = static_cast<DtuReg>(regAddr / sizeof(reg_t));

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg);
            // writes are ignored, except that the privileged flag can be changed from the outside
            else if(!isCpuRequest && reg == DtuReg::STATUS)
            {
                reg_t old = dtuRegs[static_cast<Addr>(reg)];
                set(reg, (old & ~privFlag) | (data[offset / sizeof(reg_t)] & privFlag));
            }
            else
                assert(false);
        }
        // cmd register
        else if(regAddr < sizeof(reg_t) * (numDtuRegs + numCmdRegs))
        {
            auto reg = static_cast<CmdReg>(regAddr / sizeof(reg_t) - numDtuRegs);

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg);
            else if (pkt->isWrite())
            {
                if (reg == CmdReg::COMMAND)
                    cmdChanged = true;
                set(reg, data[offset / sizeof(reg_t)]);
            }
        }
        // endpoint address
        else
        {
            unsigned epid = (regAddr - sizeof(reg_t) * (numDtuRegs + numCmdRegs)) /
                            (sizeof(reg_t) * numEpRegs);

            unsigned regNumber = (regAddr / sizeof(reg_t) - (numDtuRegs + numCmdRegs)) % numEpRegs;

            auto reg = static_cast<EpReg>(regNumber);

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(epid, reg);
            // writable only from remote and on privileged PEs
            else if(!isCpuRequest || isPrivileged)
                set(epid, reg, data[offset / sizeof(reg_t)]);
            else
                assert(false);
        }
    }

    if (pkt->needsResponse())
        pkt->makeResponse();

    return cmdChanged;
}

Addr
RegFile::getSize() const
{
    Addr size = sizeof(reg_t) * (numDtuRegs + numCmdRegs);

    size += sizeof(reg_t) * (numEndpoints * numEpRegs);

    return size;
}
