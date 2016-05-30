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
#include "debug/Dtu.hh"
#include "debug/DtuReg.hh"
#include "debug/DtuRegRange.hh"
#include "mem/dtu/regfile.hh"

#define DPRINTFNS(name, ...) do {                                       \
    Trace::getDebugLogger()->dprintf(curTick(), name, __VA_ARGS__);     \
} while (0)

const char *RegFile::dtuRegNames[] = {
    "STATUS",
    "ROOT_PT",
    "PF_EP",
    "VPE_ID",
    "MSG_CNT",
    "EXT_CMD",
};

const char *RegFile::cmdRegNames[] = {
    "COMMAND",
    "DATA_ADDR",
    "DATA_SIZE",
    "OFFSET",
    "REPLY_EPID",
    "REPLY_LABEL",
};

const char *RegFile::epTypeNames[] = {
    "INVALID",
    "SEND",
    "RECEIVE",
    "MEMORY",
    // for invalid values (the epType is 3 bits)
    "??",
    "??",
    "??",
    "??",
};

static bool isTraceEnabled(bool read)
{
    return Trace::enabled &&
        ((read && Debug::DtuRegRead) || (!read && Debug::DtuRegWrite));
}

static const char *regAccessName(RegAccess access)
{
    if (access == RegAccess::CPU)
        return "CPU";
    if (access == RegAccess::NOC)
        return "NOC";
    return "DTU";
}

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
RegFile::get(DtuReg reg, RegAccess access) const
{
    reg_t value = dtuRegs[static_cast<Addr>(reg)];

    DPRINTF(DtuRegRead, "%s<- DTU[%-12s]: %#018x\n",
                        regAccessName(access),
                        dtuRegNames[static_cast<Addr>(reg)],
                        value);

    return value;
}

void
RegFile::set(DtuReg reg, reg_t value, RegAccess access)
{
    DPRINTF(DtuRegWrite, "%s-> DTU[%-12s]: %#018x\n",
                         regAccessName(access),
                         dtuRegNames[static_cast<Addr>(reg)],
                         value);

    dtuRegs[static_cast<Addr>(reg)] = value;
}

RegFile::reg_t
RegFile::get(CmdReg reg, RegAccess access) const
{
    reg_t value = cmdRegs[static_cast<Addr>(reg)];

    DPRINTF(DtuRegRead, "%s<- CMD[%-12s]: %#018x\n",
                        regAccessName(access),
                        cmdRegNames[static_cast<Addr>(reg)],
                        value);

    return value;
}

void
RegFile::set(CmdReg reg, reg_t value, RegAccess access)
{
    DPRINTF(DtuRegWrite, "%s-> CMD[%-12s]: %#018x\n",
                         regAccessName(access),
                         cmdRegNames[static_cast<Addr>(reg)],
                         value);

    cmdRegs[static_cast<Addr>(reg)] = value;
}

EpType
RegFile::getEpType(unsigned epId) const
{
    const std::vector<reg_t> &regs = epRegs[epId];
    return static_cast<EpType>(regs[0] >> 61);
}

SendEp
RegFile::getSendEp(unsigned epId, bool print) const
{
    SendEp ep;
    if (getEpType(epId) != EpType::SEND)
    {
        DPRINTF(Dtu, "EP%u: expected SEND EP, got %s\n",
                     epId, epTypeNames[static_cast<size_t>(getEpType(epId))]);
        return ep;
    }

    const std::vector<reg_t> &regs = epRegs[epId];
    const reg_t r0  = regs[0];
    const reg_t r1  = regs[1];
    const reg_t r2  = regs[2];

    ep.vpeId        = (r0 >> 16) & 0xFFFFFFFF;
    ep.maxMsgSize   = r0 & 0xFFFF;

    ep.targetCore   = (r1 >> 24) & 0xFF;
    ep.targetEp     = (r1 >> 16) & 0xFF;
    ep.credits      = (r1 >>  0) & 0xFFFF;

    ep.label        = r2;

    if (print)
        ep.print(*this, epId, true, RegAccess::DTU);

    return ep;
}

void
RegFile::setSendEp(unsigned epId, const SendEp &ep)
{
    set(epId, 0, (static_cast<reg_t>(EpType::SEND) << 61) |
                 (ep.vpeId << 16) |
                 ep.maxMsgSize);

    set(epId, 1, (ep.targetCore << 24) | (ep.targetEp << 16) |
                 (ep.credits << 0));

    set(epId, 2, ep.label);

    ep.print(*this, epId, false, RegAccess::DTU);
}

RecvEp
RegFile::getRecvEp(unsigned epId, bool print) const
{
    RecvEp ep;
    if (getEpType(epId) != EpType::RECEIVE)
    {
        DPRINTF(Dtu, "EP%u: expected RECEIVE EP, got %s\n",
                     epId, epTypeNames[static_cast<size_t>(getEpType(epId))]);
        return ep;
    }

    const std::vector<reg_t> &regs = epRegs[epId];
    const reg_t r0  = regs[0];
    const reg_t r1  = regs[1];
    const reg_t r2  = regs[2];

    ep.msgSize      = (r0 >> 32) & 0xFFFF;
    ep.size         = (r0 >> 16) & 0xFFFF;
    ep.msgCount     = (r0 >>  0) & 0xFFFF;

    ep.bufAddr      = r1;

    ep.rdOff        = (r2 >> 16) & 0xFFFF;
    ep.wrOff        = (r2 >>  0) & 0xFFFF;

    if (print)
        ep.print(*this, epId, true, RegAccess::DTU);

    return ep;
}

void
RegFile::setRecvEp(unsigned epId, const RecvEp &ep)
{
    set(epId, 0, (static_cast<reg_t>(EpType::RECEIVE) << 61) |
                 (static_cast<reg_t>(ep.msgSize) << 32) |
                 (ep.size << 16) | (ep.msgCount << 0));

    set(epId, 1, ep.bufAddr);

    set(epId, 2, (ep.rdOff << 16) | (ep.wrOff << 0));

    ep.print(*this, epId, false, RegAccess::DTU);
}

MemEp
RegFile::getMemEp(unsigned epId, bool print) const
{
    MemEp ep;
    if (getEpType(epId) != EpType::MEMORY)
    {
        DPRINTF(Dtu, "EP%u: expected MEMORY EP, got %s\n",
                     epId, epTypeNames[static_cast<size_t>(getEpType(epId))]);
        return ep;
    }

    const std::vector<reg_t> &regs = epRegs[epId];
    const reg_t r0  = regs[0];
    const reg_t r1  = regs[1];
    const reg_t r2  = regs[2];

    ep.remoteSize   = r0 & 0x1FFFFFFFFFFFFFFF;

    ep.remoteAddr   = r1;

    ep.vpeId        = (r2 >> 12) & 0xFFFFFFFF;
    ep.targetCore   = (r2 >> 4) & 0xFF;
    ep.flags        = (r2 >> 0) & 0x7;

    if (print)
        ep.print(*this, epId, true, RegAccess::DTU);

    return ep;
}

void
SendEp::print(const RegFile &rf,
              unsigned epId,
              bool read,
              RegAccess access) const
{
    if(!isTraceEnabled(read))
        return;

    DPRINTFN(
        "%s%s EP%u%14s: Send[vpe=%u pe=%u ep=%u crd=%#x max=%#x lbl=%#llx]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "",
        vpeId, targetCore, targetEp,
        credits, maxMsgSize,
        label);
}

void
RecvEp::print(const RegFile &rf,
              unsigned epId,
              bool read,
              RegAccess access) const
{
    if(!isTraceEnabled(read))
        return;

    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Recv[buf=%p msz=%#x bsz=%#x msgs=%u rd=%#x wr=%#x]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "",
        bufAddr, msgSize, size, msgCount,
        rdOff, wrOff);
}

void
MemEp::print(const RegFile &rf,
             unsigned epId,
             bool read,
             RegAccess access) const
{
    if(!isTraceEnabled(read))
        return;

    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Mem[vpe=%u pe=%u addr=%#llx size=%#llx flags=%u]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "",
        vpeId, targetCore,
        remoteAddr, remoteSize,
        flags);
}

void
RegFile::printEpAccess(unsigned epId, bool read, bool cpu) const
{
    if (isTraceEnabled(read))
    {
        RegAccess access = cpu ? RegAccess::CPU : RegAccess::NOC;
        switch(getEpType(epId))
        {
            case EpType::SEND:
                getSendEp(epId, false).print(*this, epId, read, access);
                break;

            case EpType::RECEIVE:
                getRecvEp(epId, false).print(*this, epId, read, access);
                break;

            case EpType::MEMORY:
                getMemEp(epId, false).print(*this, epId, read, access);
                break;

            default:
            case EpType::INVALID:
                DPRINTFN("%s%s EP%u%14s: INVALID (%#x)\n",
                         regAccessName(access), read ? "<-" : "->",
                         epId, "",
                         static_cast<unsigned>(getEpType(epId)));
                break;
        }
    }
}

RegFile::reg_t
RegFile::get(unsigned epId, size_t idx) const
{
    return epRegs[epId][idx];
}

void
RegFile::set(unsigned epId, size_t idx, reg_t value)
{
    // update global message count
    if (getEpType(epId) == EpType::RECEIVE && idx == 0)
    {
        reg_t diff = (value & 0xFFFF) - (epRegs[epId][idx] & 0xFFFF);
        reg_t old = dtuRegs[static_cast<Addr>(DtuReg::MSG_CNT)];
        set(DtuReg::MSG_CNT, old + diff);
    }

    epRegs[epId][idx] = value;
}

RegFile::Result
RegFile::handleRequest(PacketPtr pkt, bool isCpuRequest)
{
    assert(pkt->isRead() || pkt->isWrite());

    Addr pktAddr = pkt->getAddr();

    DPRINTF(DtuRegRange, "access @%#x, size=%u\n", pktAddr, pkt->getSize());

    // ignore invalid accesses (might happen due to speculative execution)
    // TODO maybe we can allow some of them later
    if ((pkt->getSize() % sizeof(reg_t)) != 0 ||
        (pktAddr % sizeof(reg_t)) != 0 ||
         pktAddr + pkt->getSize() > getSize())
    {
        if (pkt->needsResponse())
            pkt->makeResponse();

        return WROTE_NONE;
    }

    // we can only perform full register accesses
    assert(pkt->getSize() % sizeof(reg_t) == 0);
    assert(pktAddr % sizeof(reg_t) == 0);
    assert(pktAddr + pkt->getSize() <= getSize());

    RegAccess access = isCpuRequest ? RegAccess::CPU : RegAccess::NOC;
    reg_t* data = pkt->getPtr<reg_t>();
    uint res = WROTE_NONE;
    reg_t privFlag = static_cast<reg_t>(Status::PRIV);
    bool isPrivileged = get(DtuReg::STATUS, RegAccess::DTU) & privFlag;
    int lastEp = -1;

    // perform a single register access for each requested register
    for (unsigned offset = 0; offset < pkt->getSize(); offset += sizeof(reg_t))
    {
        Addr regAddr = pktAddr + offset;

        // dtu register
        if (regAddr < sizeof(reg_t) * numDtuRegs)
        {
            auto reg = static_cast<DtuReg>(regAddr / sizeof(reg_t));

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg, access);
            // MSG_CNT can't be set in general; all can't be set by the CPU
            else if (pkt->isWrite() && !isCpuRequest && reg != DtuReg::MSG_CNT)
            {
                if (reg == DtuReg::EXT_CMD)
                    res |= WROTE_EXT_CMD;
                set(reg, data[offset / sizeof(reg_t)], access);
            }
            else
                assert(false);
        }
        // cmd register
        else if (regAddr < sizeof(reg_t) * (numDtuRegs + numCmdRegs))
        {
            size_t idx = regAddr / sizeof(reg_t) - numDtuRegs;
            auto reg = static_cast<CmdReg>(idx);

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg, access);
            else if (pkt->isWrite())
            {
                if (reg == CmdReg::COMMAND)
                    res |= WROTE_CMD;
                set(reg, data[offset / sizeof(reg_t)], access);
            }
        }
        // endpoint address
        else
        {
            size_t nonEpRegs = numDtuRegs + numCmdRegs;
            unsigned epId = (regAddr - sizeof(reg_t) * nonEpRegs) /
                            (sizeof(reg_t) * numEpRegs);

            unsigned regNumber = (regAddr / sizeof(reg_t) -
                                 nonEpRegs) % numEpRegs;

            if (lastEp != epId)
            {
                if (lastEp != -1)
                    printEpAccess(lastEp, pkt->isRead(), isCpuRequest);
                lastEp = epId;
            }

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(epId, regNumber);
            // writable only from remote and on privileged PEs
            else if (!isCpuRequest || isPrivileged)
                set(epId, regNumber, data[offset / sizeof(reg_t)]);
            else
                assert(false);
        }
    }

    if (lastEp != -1)
        printEpAccess(lastEp, pkt->isRead(), isCpuRequest);

    if (pkt->needsResponse())
        pkt->makeResponse();

    return static_cast<Result>(res);
}

Addr
RegFile::getSize() const
{
    Addr size = sizeof(reg_t) * (numDtuRegs + numCmdRegs);

    size += sizeof(reg_t) * (numEndpoints * numEpRegs);

    return size;
}
