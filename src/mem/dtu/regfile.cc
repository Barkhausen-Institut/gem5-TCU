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
#include "mem/dtu/dtu.hh"

#define DPRINTFNS(name, ...) do {                                       \
    Trace::getDebugLogger()->dprintf(curTick(), name, __VA_ARGS__);     \
} while (0)

const char *RegFile::dtuRegNames[] = {
    "FEATURES",
    "ROOT_PT",
    "PF_EP",
    "CUR_TIME",
    "EVENTS",
    "EXT_CMD",
    "CLEAR_IRQ",
    "CLOCK",
};

const char *RegFile::reqRegNames[] = {
    "EXT_REQ",
    "XLATE_REQ",
    "XLATE_RESP",
};

const char *RegFile::cmdRegNames[] = {
    "COMMAND",
    "ABORT",
    "DATA",
    "OFFSET",
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
    return (read && DTRACE(DtuRegRead)) || (!read && DTRACE(DtuRegWrite));
}

static const char *regAccessName(RegAccess access)
{
    if (access == RegAccess::CPU)
        return "CPU";
    if (access == RegAccess::NOC)
        return "NOC";
    return "DTU";
}

RegFile::RegFile(Dtu &_dtu, const std::string& name, unsigned _numEndpoints)
    : dtu(_dtu),
      dtuRegs(numDtuRegs, 0),
      reqRegs(numReqRegs, 0),
      cmdRegs(numCmdRegs, 0),
      epRegs(_numEndpoints),
      bufRegs(numBufRegs * sizeof(reg_t), 0),
      numEndpoints(_numEndpoints),
      _name(name)
{
    // at boot, all PEs are privileged
    reg_t feat = static_cast<reg_t>(Features::PRIV);
    set(DtuReg::FEATURES, feat);

    for (int epid = 0; epid < numEndpoints; epid++)
    {
        for (int i = 0; i < numEpRegs; i++)
            epRegs[epid].push_back(0);
    }
}

bool
RegFile::invalidate(unsigned epId, bool force)
{
    if (!force && getEpType(epId) == EpType::SEND)
    {
        SendEp sep = getSendEp(epId);
        if (sep.curcrd != sep.maxcrd)
            return false;
    }

    for (int i = 0; i < numEpRegs; ++i)
        epRegs[epId][i] = 0;

    printEpAccess(epId, false, false);

    return true;
}

void
RegFile::setEvent(EventType ev)
{
    reg_t old = dtuRegs[static_cast<size_t>(DtuReg::EVENTS)];
    set(DtuReg::EVENTS,
        old | static_cast<reg_t>(1) << static_cast<reg_t>(ev),
        RegAccess::DTU);
}

bool
RegFile::ackEvents(reg_t old)
{
    if (get(DtuReg::EVENTS, RegAccess::DTU) == old)
    {
        dtuRegs[static_cast<size_t>(DtuReg::EVENTS)] = 0;
        updateMsgCnt();
        return true;
    }
    return false;
}

RegFile::reg_t
RegFile::get(DtuReg reg, RegAccess access) const
{
    reg_t value;

    if (reg == DtuReg::CUR_TIME)
        value = dtu.curCycle();
    else if (reg == DtuReg::CLOCK)
        value = dtu.frequency();
    else
        value = dtuRegs[static_cast<Addr>(reg)];

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
RegFile::get(ReqReg reg, RegAccess access) const
{
    reg_t value = reqRegs[static_cast<Addr>(reg)];

    DPRINTF(DtuRegRead, "%s<- REQ[%-12s]: %#018x\n",
                        regAccessName(access),
                        reqRegNames[static_cast<Addr>(reg)],
                        value);

    return value;
}

void
RegFile::set(ReqReg reg, reg_t value, RegAccess access)
{
    DPRINTF(DtuRegWrite, "%s-> REQ[%-12s]: %#018x\n",
                         regAccessName(access),
                         reqRegNames[static_cast<Addr>(reg)],
                         value);

    reqRegs[static_cast<Addr>(reg)] = value;
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

    ep.flags        = (r0 >> 26) & 0x3;
    ep.crdEp        = (r0 >> 18) & 0xFF;
    ep.maxMsgSize   = (r0 >> 12) & 0x3F;
    ep.maxcrd       = (r0 >>  6) & 0x3F;
    ep.curcrd       = (r0 >>  0) & 0x3F;

    ep.targetCore   = (r1 >>  8) & 0xFF;
    ep.targetEp     = (r1 >>  0) & 0xFF;

    ep.label        = r2;

    if (print)
        ep.print(*this, epId, true, RegAccess::DTU);

    return ep;
}

void
RegFile::setSendEp(unsigned epId, const SendEp &ep)
{
    set(epId, 0, (static_cast<reg_t>(EpType::SEND) << 61) |
                 (static_cast<reg_t>(ep.flags)      << 26) |
                 (static_cast<reg_t>(ep.crdEp)      << 18) |
                 (static_cast<reg_t>(ep.maxMsgSize) << 12) |
                 (static_cast<reg_t>(ep.maxcrd)     << 6) |
                 (static_cast<reg_t>(ep.curcrd)     << 0));

    set(epId, 1, (static_cast<reg_t>(ep.targetCore) << 8) |
                 (static_cast<reg_t>(ep.targetEp)   << 0));

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

    ep.rdPos        = (r0 >> 54) & 0x3F;
    ep.wrPos        = (r0 >> 48) & 0x3F;
    ep.msgSize      = (r0 >> 32) & 0xFFFF;
    ep.size         = (r0 >> 26) & 0x3F;
    ep.replyEps     = (r0 >>  6) & 0xFFFFF;
    ep.msgCount     = (r0 >>  0) & 0x3F;

    ep.bufAddr      = r1;

    ep.occupied     = r2 & 0xFFFFFFFF;
    ep.unread       = r2 >> 32;

    if (print)
        ep.print(*this, epId, true, RegAccess::DTU);

    return ep;
}

void
RegFile::setRecvEp(unsigned epId, const RecvEp &ep)
{
    set(epId, 0, (static_cast<reg_t>(EpType::RECEIVE) << 61) |
                 (static_cast<reg_t>(ep.rdPos)        << 54) |
                 (static_cast<reg_t>(ep.wrPos)        << 48) |
                 (static_cast<reg_t>(ep.msgSize)      << 32) |
                 (static_cast<reg_t>(ep.size)         << 26) |
                 (static_cast<reg_t>(ep.replyEps)     << 6) |
                 (static_cast<reg_t>(ep.msgCount)     << 0));

    set(epId, 1, ep.bufAddr);

    set(epId, 2, (static_cast<reg_t>(ep.unread)       << 32) |
                 (static_cast<reg_t>(ep.occupied)     << 0));

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

    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Send[pe=%u ep=%u crdep=%u maxcrd=%u curcrd=%u max=%#x lbl=%#llx fl=%#lx]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "",
        targetCore, targetEp, crdEp,
        maxcrd, curcrd, 1 << maxMsgSize,
        label, flags);
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
        "%s%s EP%u%14s: Recv[buf=%p msz=%#x bsz=%#x rpl=%u msgs=%u occ=%#010x unr=%#010x rd=%u wr=%u]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "",
        bufAddr, 1 << msgSize, 1 << size, replyEps, msgCount,
        occupied, unread, rdPos, wrPos);
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
        "%s%s EP%u%14s: Mem[pe=%u addr=%#llx size=%#llx flags=%#x]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "",
        targetCore,
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
    bool oldrecv = getEpType(epId) == EpType::RECEIVE;
    bool newrecv = static_cast<EpType>(value >> 61) == EpType::RECEIVE;

    epRegs[epId][idx] = value;

    // update global message count, if it was a receive EP or got a receive EP
    if (idx == 0 && (newrecv || oldrecv))
        updateMsgCnt();
}

void
RegFile::updateMsgCnt()
{
    bool hasMsgs = false;
    for (size_t i = 0; i < dtu.numEndpoints; ++i) {
        if (getEpType(i) == EpType::RECEIVE && (get(i, 0) & 0x3F) > 0) {
            hasMsgs = true;
            break;
        }
    }

    reg_t other = dtuRegs[static_cast<size_t>(DtuReg::EVENTS)];
    other &= ~static_cast<reg_t>(1);
    reg_t msgs = (hasMsgs ? 1 : 0) << static_cast<reg_t>(EventType::MSG_RECV);
    set(DtuReg::EVENTS, other | msgs);
}

const char *
RegFile::getBuffer(size_t bytes)
{
    static char tmp[256 + 1];
    assert(bytes + 1 <= sizeof(tmp));
    memcpy(tmp, bufRegs.data(), bytes);
    tmp[bytes] = '\0';
    return tmp;
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
    bool isPriv = hasFeature(Features::PRIV);
    int lastEp = -1;

    // perform a single register access for each requested register
    for (unsigned offset = 0; offset < pkt->getSize(); offset += sizeof(reg_t))
    {
        Addr regAddr = pktAddr + offset;

        // master register
        if (regAddr < sizeof(reg_t) * numDtuRegs)
        {
            auto reg = static_cast<DtuReg>(regAddr / sizeof(reg_t));

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg, access);
            else if (pkt->isWrite() && isCpuRequest && reg == DtuReg::CLEAR_IRQ)
                res |= WROTE_CLEAR_IRQ;
            // master registers can't be set by the CPU
            else if (pkt->isWrite() && !isCpuRequest)
            {
                if (reg == DtuReg::EXT_CMD)
                    res |= WROTE_EXT_CMD;
                set(reg, data[offset / sizeof(reg_t)], access);
            }
        }
        else if (regAddr >= DtuTlb::PAGE_SIZE)
        {
            Addr reqAddr = regAddr - DtuTlb::PAGE_SIZE;

            if (reqAddr < sizeof(reg_t) * numReqRegs)
            {
                auto reg = static_cast<ReqReg>(reqAddr / sizeof(reg_t));

                if (pkt->isRead())
                    data[offset / sizeof(reg_t)] = get(reg, access);
                // privileged registers can only be set if we're privileged
                else if (pkt->isWrite())
                {
                    if (reg == ReqReg::XLATE_REQ || reg == ReqReg::XLATE_RESP)
                        res |= WROTE_XLATE;
                    // it only triggers an IRQ if the value is non-zero
                    else if (data[offset / sizeof(reg_t)] != 0)
                        res |= WROTE_EXT_REQ;
                    set(reg, data[offset / sizeof(reg_t)], access);
                }
            }
        }
        // cmd register
        else if (regAddr < sizeof(reg_t) * (numDtuRegs + numCmdRegs))
        {
            size_t idx = regAddr / sizeof(reg_t) - numDtuRegs;
            auto reg = static_cast<CmdReg>(idx);

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg, access);
            // the command register can't be written from the NoC
            else if (pkt->isWrite() && (reg != CmdReg::COMMAND || isCpuRequest))
            {
                if (reg == CmdReg::COMMAND)
                    res |= WROTE_CMD;
                else if (reg == CmdReg::ABORT)
                    res |= WROTE_ABORT;
                set(reg, data[offset / sizeof(reg_t)], access);
            }
        }
        else
        {
            size_t nonEpRegs = numDtuRegs + numCmdRegs;
            Addr epAddr = regAddr - sizeof(reg_t) * nonEpRegs;

            // endpoint address
            if (epAddr < sizeof(reg_t) * numEpRegs * dtu.numEndpoints)
            {
                unsigned epId = epAddr / (sizeof(reg_t) * numEpRegs);
                unsigned regNumber = (epAddr / sizeof(reg_t)) % numEpRegs;

                if (lastEp != epId)
                {
                    if (lastEp != -1)
                        printEpAccess(lastEp, pkt->isRead(), isCpuRequest);
                    lastEp = epId;
                }

                if (pkt->isRead())
                    data[offset / sizeof(reg_t)] = get(epId, regNumber);
                // writable only from remote and master DTUs
                else if (!isCpuRequest || isPriv)
                    set(epId, regNumber, data[offset / sizeof(reg_t)]);
                else
                    assert(false);
            }
            // buf register
            else
            {
                Addr bufAddr = epAddr -
                    sizeof(reg_t) * numEpRegs * dtu.numEndpoints;
                size_t idx = bufAddr / sizeof(reg_t);

                if (idx < bufRegs.size())
                {
                    if(pkt->isRead())
                        data[offset / sizeof(reg_t)] = bufRegs[idx];
                    else
                        bufRegs[idx] = data[offset / sizeof(reg_t)];
                }
            }
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
    return DtuTlb::PAGE_SIZE + sizeof(reg_t) * numReqRegs;
}
