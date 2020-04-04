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
#include "debug/Tcu.hh"
#include "debug/TcuReg.hh"
#include "debug/TcuRegRange.hh"
#include "mem/tcu/regfile.hh"
#include "mem/tcu/tcu.hh"

#define DPRINTFNS(name, ...) do {                                       \
    Trace::getDebugLogger()->dprintf(curTick(), name, __VA_ARGS__);     \
} while (0)

const char *RegFile::tcuRegNames[] = {
    "FEATURES",
    "CUR_TIME",
    "CLEAR_IRQ",
    "CLOCK",
};

const char *RegFile::privRegNames[] = {
    "CORE_REQ",
    "CORE_RESP",
    "PRIV_CMD",
    "PRIV_CMD_ARG",
    "EXT_CMD",
    "CUR_VPE",
    "OLD_VPE",
};

const char *RegFile::cmdRegNames[] = {
    "COMMAND",
    "ABORT",
    "DATA",
    "ARG1",
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
    return (read && DTRACE(TcuRegRead)) || (!read && DTRACE(TcuRegWrite));
}

static const char *regAccessName(RegAccess access)
{
    if (access == RegAccess::CPU)
        return "CPU";
    if (access == RegAccess::NOC)
        return "NOC";
    return "TCU";
}

RegFile::RegFile(Tcu &_tcu, const std::string& name, unsigned _numEndpoints)
    : tcu(_tcu),
      tcuRegs(numTcuRegs, 0),
      privRegs(numPrivRegs, 0),
      cmdRegs(numCmdRegs, 0),
      epRegs(_numEndpoints),
      bufRegs(numBufRegs * sizeof(reg_t), 0),
      numEndpoints(_numEndpoints),
      _name(name)
{
    static_assert(sizeof(tcuRegNames) / sizeof(tcuRegNames[0]) ==
        numTcuRegs, "tcuRegNames out of sync");
    static_assert(sizeof(privRegNames) / sizeof(privRegNames[0]) ==
        numPrivRegs, "privRegNames out of sync");
    static_assert(sizeof(cmdRegNames) / sizeof(cmdRegNames[0]) ==
        numCmdRegs, "cmdRegNames out of sync");

    for (epid_t epid = 0; epid < numEndpoints; epid++)
    {
        for (int i = 0; i < numEpRegs; i++)
            epRegs[epid].push_back(0);
    }

    // at boot, all PEs are privileged
    reg_t feat = static_cast<reg_t>(Features::PRIV);
    set(TcuReg::FEATURES, feat);
    set(PrivReg::CUR_VPE, static_cast<reg_t>(Tcu::INVALID_VPE_ID) << 19);
}

unsigned
RegFile::countMsgs(vpeid_t vpeId)
{
    unsigned count = 0;
    for (epid_t epid = 0; epid < numEndpoints; epid++)
    {
        if (getEpType(epid) == EpType::RECEIVE)
        {
            RecvEp rep = getRecvEp(epid);
            if (rep.vpe == vpeId)
                count += rep.unreadMsgs();
        }
    }
    return count;
}

TcuError
RegFile::invalidate(epid_t epId, bool force, unsigned *unreadMask)
{
    *unreadMask = 0;

    if (!force && getEpType(epId) == EpType::SEND)
    {
        SendEp sep = getSendEp(epId);
        if (sep.curcrd != sep.maxcrd)
            return TcuError::MISS_CREDITS;
    }

    if (!force && getEpType(epId) == EpType::RECEIVE)
    {
        RecvEp rep = getRecvEp(epId);
        *unreadMask = rep.unread;
    }

    for (int i = 0; i < numEpRegs; ++i)
        epRegs[epId][i] = 0;

    printEpAccess(epId, false, false);

    return TcuError::NONE;
}

void
RegFile::add_msg()
{
    reg_t cur_vpe = privRegs[static_cast<size_t>(PrivReg::CUR_VPE)];
    cur_vpe += 1 << 3;
    set(PrivReg::CUR_VPE, cur_vpe);
    // since we're always invalidating receive EPs immediately and do NOT
    // update the message count in the CUR_VPE reg, we might temporarily have
    // less pending messages than CUR_VPE indicates.
    assert(messages() >= countMsgs(getVPE()));
}

void
RegFile::rem_msg()
{
    reg_t cur_vpe = privRegs[static_cast<size_t>(PrivReg::CUR_VPE)];
    cur_vpe -= 1 << 3;
    set(PrivReg::CUR_VPE, cur_vpe);
    assert(messages() >= countMsgs(getVPE()));
}

RegFile::reg_t
RegFile::fetchEvents()
{
    reg_t old = get(PrivReg::CUR_VPE, RegAccess::TCU);
    if ((old & 0x7) != 0)
        set(PrivReg::CUR_VPE, old & ~static_cast<reg_t>(0x7));
    return old & 0x7;
}

void
RegFile::setEvent(EventType ev)
{
    reg_t old = privRegs[static_cast<size_t>(PrivReg::CUR_VPE)];
    set(PrivReg::CUR_VPE,
        old | static_cast<reg_t>(1) << static_cast<reg_t>(ev),
        RegAccess::TCU);
}

RegFile::reg_t
RegFile::get(TcuReg reg, RegAccess access) const
{
    reg_t value;

    if (reg == TcuReg::CUR_TIME)
        value = tcu.curCycle();
    else if (reg == TcuReg::CLOCK)
        value = tcu.frequency();
    else
        value = tcuRegs[static_cast<Addr>(reg)];

    DPRINTF(TcuRegRead, "%s<- TCU[%-12s]: %#018x\n",
                        regAccessName(access),
                        tcuRegNames[static_cast<Addr>(reg)],
                        value);

    return value;
}

void
RegFile::set(TcuReg reg, reg_t value, RegAccess access)
{
    DPRINTF(TcuRegWrite, "%s-> TCU[%-12s]: %#018x\n",
                         regAccessName(access),
                         tcuRegNames[static_cast<Addr>(reg)],
                         value);

    tcuRegs[static_cast<Addr>(reg)] = value;
}

RegFile::reg_t
RegFile::get(PrivReg reg, RegAccess access) const
{
    reg_t value = privRegs[static_cast<Addr>(reg)];

    DPRINTF(TcuRegRead, "%s<- PRI[%-12s]: %#018x\n",
                        regAccessName(access),
                        privRegNames[static_cast<Addr>(reg)],
                        value);

    return value;
}

void
RegFile::set(PrivReg reg, reg_t value, RegAccess access)
{
    DPRINTF(TcuRegWrite, "%s-> PRI[%-12s]: %#018x\n",
                         regAccessName(access),
                         privRegNames[static_cast<Addr>(reg)],
                         value);

    privRegs[static_cast<Addr>(reg)] = value;
}

RegFile::reg_t
RegFile::get(CmdReg reg, RegAccess access) const
{
    reg_t value = cmdRegs[static_cast<Addr>(reg)];

    DPRINTF(TcuRegRead, "%s<- CMD[%-12s]: %#018x\n",
                        regAccessName(access),
                        cmdRegNames[static_cast<Addr>(reg)],
                        value);

    return value;
}

void
RegFile::set(CmdReg reg, reg_t value, RegAccess access)
{
    DPRINTF(TcuRegWrite, "%s-> CMD[%-12s]: %#018x\n",
                         regAccessName(access),
                         cmdRegNames[static_cast<Addr>(reg)],
                         value);

    cmdRegs[static_cast<Addr>(reg)] = value;
}

EpType
RegFile::getEpType(epid_t epId) const
{
    const std::vector<reg_t> &regs = epRegs[epId];
    return static_cast<EpType>(regs[0] & 0x7);
}

SendEp
RegFile::getSendEp(epid_t epId, bool print) const
{
    SendEp ep;
    if (getEpType(epId) != EpType::SEND)
    {
        DPRINTF(Tcu, "EP%u: expected SEND EP, got %s\n",
                     epId, epTypeNames[static_cast<size_t>(getEpType(epId))]);
        return ep;
    }

    const std::vector<reg_t> &regs = epRegs[epId];
    const reg_t r0  = regs[0];
    const reg_t r1  = regs[1];
    const reg_t r2  = regs[2];

    ep.flags        = (r0 >> 53) & 0x3;
    ep.crdEp        = (r0 >> 37) & 0xFFFF;
    ep.maxMsgSize   = (r0 >> 31) & 0x3F;
    ep.maxcrd       = (r0 >> 25) & 0x3F;
    ep.curcrd       = (r0 >> 19) & 0x3F;
    ep.vpe          = (r0 >> 3) & 0xFFFF;

    ep.targetPe     = (r1 >>  16) & 0xFF;
    ep.targetEp     = (r1 >>  0) & 0xFFFF;

    ep.label        = r2 & 0xFFFFFFFF;

    if (print)
        ep.print(*this, epId, true, RegAccess::TCU);

    return ep;
}

void
RegFile::setSendEp(epid_t epId, const SendEp &ep)
{
    set(epId, 0, (static_cast<reg_t>(ep.flags)      << 53) |
                 (static_cast<reg_t>(ep.crdEp)      << 37) |
                 (static_cast<reg_t>(ep.maxMsgSize) << 31) |
                 (static_cast<reg_t>(ep.maxcrd)     << 25) |
                 (static_cast<reg_t>(ep.curcrd)     << 19) |
                 (static_cast<reg_t>(ep.vpe)        << 3) |
                 (static_cast<reg_t>(EpType::SEND)  << 0));

    set(epId, 1, (static_cast<reg_t>(ep.targetPe) << 16) |
                 (static_cast<reg_t>(ep.targetEp)   << 0));

    set(epId, 2, ep.label);

    ep.print(*this, epId, false, RegAccess::TCU);
}

RecvEp
RegFile::getRecvEp(epid_t epId, bool print) const
{
    RecvEp ep;
    if (getEpType(epId) != EpType::RECEIVE)
    {
        DPRINTF(Tcu, "EP%u: expected RECEIVE EP, got %s\n",
                     epId, epTypeNames[static_cast<size_t>(getEpType(epId))]);
        return ep;
    }

    const std::vector<reg_t> &regs = epRegs[epId];
    const reg_t r0  = regs[0];
    const reg_t r1  = regs[1];
    const reg_t r2  = regs[2];

    ep.rdPos        = (r0 >> 53) & 0x3F;
    ep.wrPos        = (r0 >> 47) & 0x3F;
    ep.msgSize      = (r0 >> 41) & 0x3F;
    ep.size         = (r0 >> 35) & 0x3F;
    ep.replyEps     = (r0 >> 19) & 0xFFFF;
    ep.vpe          = (r0 >>  3) & 0xFFFF;

    ep.bufAddr      = r1 & 0xFFFFFFFF;

    ep.occupied     = r2 & 0xFFFFFFFF;
    ep.unread       = r2 >> 32;

    if (print)
        ep.print(*this, epId, true, RegAccess::TCU);

    return ep;
}

void
RegFile::setRecvEp(epid_t epId, const RecvEp &ep)
{
    set(epId, 0, (static_cast<reg_t>(ep.rdPos)        << 53) |
                 (static_cast<reg_t>(ep.wrPos)        << 47) |
                 (static_cast<reg_t>(ep.msgSize)      << 41) |
                 (static_cast<reg_t>(ep.size)         << 35) |
                 (static_cast<reg_t>(ep.replyEps)     << 19) |
                 (static_cast<reg_t>(ep.vpe)          << 3) |
                 (static_cast<reg_t>(EpType::RECEIVE) << 0));

    set(epId, 1, ep.bufAddr);

    set(epId, 2, (static_cast<reg_t>(ep.unread)       << 32) |
                 (static_cast<reg_t>(ep.occupied)     << 0));

    ep.print(*this, epId, false, RegAccess::TCU);
}

MemEp
RegFile::getMemEp(epid_t epId, bool print) const
{
    MemEp ep;
    if (getEpType(epId) != EpType::MEMORY)
    {
        DPRINTF(Tcu, "EP%u: expected MEMORY EP, got %s\n",
                     epId, epTypeNames[static_cast<size_t>(getEpType(epId))]);
        return ep;
    }

    const std::vector<reg_t> &regs = epRegs[epId];
    const reg_t r0  = regs[0];
    const reg_t r1  = regs[1];
    const reg_t r2  = regs[2];

    ep.targetVpe    = (r0 >> 31) & 0xFFFF;
    ep.targetPe     = (r0 >> 23) & 0xFF;
    ep.flags        = (r0 >> 19) & 0x7;
    ep.vpe          = (r0 >> 3) & 0xFFFF;

    ep.remoteAddr   = r1;

    ep.remoteSize   = r2;

    if (print)
        ep.print(*this, epId, true, RegAccess::TCU);

    return ep;
}

void
SendEp::print(const RegFile &rf,
              epid_t epId,
              bool read,
              RegAccess access) const
{
    if(!isTraceEnabled(read))
        return;

    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Send[vpe=%u, pe=%u ep=%u crdep=%u maxcrd=%u curcrd=%u max=%#x lbl=%#llx fl=%#lx]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "", vpe,
        targetPe, targetEp, crdEp,
        maxcrd, curcrd, 1 << maxMsgSize,
        label, flags);
}

void
RecvEp::print(const RegFile &rf,
              epid_t epId,
              bool read,
              RegAccess access) const
{
    if(!isTraceEnabled(read))
        return;

    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Recv[vpe=%u, buf=%p msz=%#x bsz=%#x rpl=%u msgs=%u occ=%#010x unr=%#010x rd=%u wr=%u]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "", vpe,
        bufAddr, 1 << msgSize, 1 << size, replyEps,
        unreadMsgs(), occupied, unread, rdPos, wrPos);
}

void
MemEp::print(const RegFile &rf,
             epid_t epId,
             bool read,
             RegAccess access) const
{
    if(!isTraceEnabled(read))
        return;

    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Mem[vpe=%u, pe=%u tvpe=%u addr=%#llx size=%#llx flags=%#x]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "", vpe,
        targetPe, targetVpe,
        remoteAddr, remoteSize,
        flags);
}

void
RegFile::printEpAccess(epid_t epId, bool read, bool cpu) const
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
RegFile::get(epid_t epId, size_t idx) const
{
    return epRegs[epId][idx];
}

void
RegFile::set(epid_t epId, size_t idx, reg_t value)
{
    epRegs[epId][idx] = value;
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

    DPRINTF(TcuRegRange, "access @%#x, size=%u\n", pktAddr, pkt->getSize());

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
        if (regAddr < sizeof(reg_t) * numTcuRegs)
        {
            auto reg = static_cast<TcuReg>(regAddr / sizeof(reg_t));

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg, access);
            else if (pkt->isWrite() && isCpuRequest && reg == TcuReg::CLEAR_IRQ)
                res |= WROTE_CLEAR_IRQ;
            // master registers can't be set by the CPU
            else if (pkt->isWrite() && !isCpuRequest)
                set(reg, data[offset / sizeof(reg_t)], access);
        }
        else if (regAddr >= TcuTlb::PAGE_SIZE * 2)
        {
            Addr reqAddr = regAddr - TcuTlb::PAGE_SIZE * 2;

            if (reqAddr < sizeof(reg_t) * numPrivRegs)
            {
                auto reg = static_cast<PrivReg>(reqAddr / sizeof(reg_t));

                if (pkt->isRead())
                    data[offset / sizeof(reg_t)] = get(reg, access);
                // privileged registers can only be set if we're privileged
                else if (pkt->isWrite())
                {
                    if (reg == PrivReg::CORE_REQ || reg == PrivReg::CORE_RESP)
                        res |= WROTE_XLATE;
                    else if (reg == PrivReg::PRIV_CMD)
                        res |= WROTE_PRIV_CMD;
                    // EXT_CMD can only be written externally
                    else if (reg == PrivReg::EXT_CMD && !isCpuRequest)
                        res |= WROTE_EXT_CMD;
                    if (reg != PrivReg::EXT_CMD || !isCpuRequest)
                        set(reg, data[offset / sizeof(reg_t)], access);
                }
            }
        }
        // cmd register
        else if (regAddr < sizeof(reg_t) * (numTcuRegs + numCmdRegs))
        {
            size_t idx = regAddr / sizeof(reg_t) - numTcuRegs;
            auto reg = static_cast<CmdReg>(idx);

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg, access);
            // the command registers can't be written from the NoC
            else if (pkt->isWrite() && isCpuRequest)
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
            size_t nonEpRegs = numTcuRegs + numCmdRegs;
            Addr epAddr = regAddr - sizeof(reg_t) * nonEpRegs;

            // endpoint address
            if (epAddr < sizeof(reg_t) * numEpRegs * tcu.numEndpoints)
            {
                epid_t epId = epAddr / (sizeof(reg_t) * numEpRegs);
                unsigned regNumber = (epAddr / sizeof(reg_t)) % numEpRegs;

                if (lastEp != epId)
                {
                    if (lastEp != -1)
                        printEpAccess(lastEp, pkt->isRead(), isCpuRequest);
                    lastEp = epId;
                }

                if (pkt->isRead())
                    data[offset / sizeof(reg_t)] = get(epId, regNumber);
                // writable only from remote and master TCUs
                else if (!isCpuRequest || isPriv)
                    set(epId, regNumber, data[offset / sizeof(reg_t)]);
                else
                    assert(false);
            }
            // buf register
            else
            {
                Addr bufAddr = epAddr -
                    sizeof(reg_t) * numEpRegs * tcu.numEndpoints;
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
    return TcuTlb::PAGE_SIZE * 2 + sizeof(reg_t) * numPrivRegs;
}
