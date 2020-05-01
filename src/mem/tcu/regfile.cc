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
    "PRINT",
};

const char *RegFile::privRegNames[] = {
    "CORE_REQ",
    "PRIV_CMD",
    "PRIV_CMD_ARG",
    "EXT_CMD",
    "CUR_VPE",
    "OLD_VPE",
};

const char *RegFile::cmdRegNames[] = {
    "COMMAND",
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
      eps(_numEndpoints, Ep()),
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

    // at boot, all PEs are privileged
    reg_t feat = static_cast<reg_t>(Features::PRIV);
    set(TcuReg::FEATURES, feat);

    // and no VPE is running (the id might stay invalid for PEs that don't
    // support multiple VPEs though)
    VPEState vpe = 0;
    vpe.id = Tcu::INVALID_VPE_ID;
    vpe.msgs = 0;
    set(PrivReg::CUR_VPE, vpe);
}

TcuError
RegFile::invalidate(epid_t epId, bool force, unsigned *unreadMask)
{
    *unreadMask = 0;

    SendEp *sep;
    if (!force && (sep = getSendEp(epId)) != nullptr)
    {
        if (sep->r0.curcrd != sep->r0.maxcrd)
            return TcuError::MISS_CREDITS;
    }

    RecvEp *rep;
    if (!force && (rep = getRecvEp(epId)) != nullptr)
        *unreadMask = rep->r2.unread;

    for (int i = 0; i < numEpRegs; ++i)
        set(epId, i, 0);

    updateEp(epId);

    return TcuError::NONE;
}

void
RegFile::add_msg()
{
    VPEState cur = getVPE(PrivReg::CUR_VPE);
    cur.msgs = cur.msgs + 1;
    set(PrivReg::CUR_VPE, cur);
}

void
RegFile::rem_msg()
{
    VPEState cur = getVPE(PrivReg::CUR_VPE);
    cur.msgs = cur.msgs - 1;
    set(PrivReg::CUR_VPE, cur);
}

RegFile::reg_t
RegFile::get(TcuReg reg, RegAccess access) const
{
    reg_t value;

    if (reg == TcuReg::CUR_TIME)
        value = curTick() / 1000;
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

SendEp*
RegFile::getSendEp(epid_t epId, bool print)
{
    Ep *ep = getEp(epId, EpType::SEND, print);
    return ep ? &ep->send : nullptr;
}

RecvEp*
RegFile::getRecvEp(epid_t epId, bool print)
{
    Ep *ep = getEp(epId, EpType::RECEIVE, print);
    return ep ? &ep->recv : nullptr;
}

MemEp*
RegFile::getMemEp(epid_t epId, bool print)
{
    Ep *ep = getEp(epId, EpType::MEMORY, print);
    return ep ? &ep->mem : nullptr;
}

Ep*
RegFile::getEp(epid_t epId, EpType type, bool print)
{
    Ep &ep = eps.at(epId);
    if (ep.type() != type)
    {
        DPRINTF(Tcu, "EP%u: expected %s EP, got %s\n",
                epId, epTypeNames[static_cast<size_t>(type)],
                epTypeNames[static_cast<size_t>(ep.type())]);
        return nullptr;
    }

    if (print)
        printEpAccess(epId, true, RegAccess::TCU);
    return &ep;
}

void
RegFile::updateEp(epid_t epId)
{
    printEpAccess(epId, false, RegAccess::TCU);
}

void
RegFile::printEpAccess(epid_t epId, bool read, RegAccess access) const
{
    if(!isTraceEnabled(read))
        return;

    const Ep &ep = eps.at(epId);
    switch (ep.type())
    {
        case EpType::INVALID:
            ep.inval.print(*this, epId, false, access);
            break;
        case EpType::SEND:
            ep.send.print(*this, epId, false, access);
            break;
        case EpType::RECEIVE:
            ep.recv.print(*this, epId, false, access);
            break;
        case EpType::MEMORY:
            ep.mem.print(*this, epId, false, access);
            break;
    }
}

void
SendEp::print(const RegFile &rf,
              epid_t epId,
              bool read,
              RegAccess access) const
{
    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Send[vpe=%u, pe=%u ep=%u crdep=%u maxcrd=%u "
                            "curcrd=%u max=%#x lbl=%#llx fl=%#lx]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "", r0.vpe,
        r1.targetPe, r1.targetEp, r0.crdEp,
        r0.maxcrd, r0.curcrd, 1 << r0.maxMsgSize,
        r2.label, r0.flags);
}

void
RecvEp::print(const RegFile &rf,
              epid_t epId,
              bool read,
              RegAccess access) const
{
    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Recv[vpe=%u, buf=%p msz=%#x bsz=%#x rpl=%u msgs=%u "
                            "occ=%#010x unr=%#010x rd=%u wr=%u]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "", r0.vpe,
        r1.bufAddr, 1 << r0.msgSize, 1 << r0.size, r0.replyEps,
        unreadMsgs(), r2.occupied, r2.unread, r0.rdPos, r0.wrPos);
}

void
MemEp::print(const RegFile &rf,
             epid_t epId,
             bool read,
             RegAccess access) const
{
    DPRINTFNS(rf.name(),
        "%s%s EP%u%14s: Mem[vpe=%u, pe=%u tvpe=%u addr=%#llx "
                           "size=%#llx flags=%#x]\n",
        regAccessName(access), read ? "<-" : "->",
        epId, "", r0.vpe,
        r0.targetPe, r0.targetVpe,
        r1.remoteAddr, r2.remoteSize,
        r0.flags);
}

void
InvalidEp::print(const RegFile &rf,
                 epid_t epId,
                 bool read,
                 RegAccess access) const
{
    DPRINTFN("%s%s EP%u%14s: INVALID (%#x)\n",
             regAccessName(access), read ? "<-" : "->",
             epId, "",
             static_cast<uint>(type()));
}

RegFile::reg_t
RegFile::get(epid_t epId, size_t idx) const
{
    return eps[epId].inval.r[idx];
}

void
RegFile::set(epid_t epId, size_t idx, reg_t value)
{
    eps[epId].inval.r[idx] = value;
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
            {
                res |= WROTE_CLEAR_IRQ;
                set(reg, data[offset / sizeof(reg_t)], access);
            }
            else if (pkt->isWrite() && isCpuRequest && reg == TcuReg::PRINT)
            {
                res |= WROTE_PRINT;
                set(reg, data[offset / sizeof(reg_t)], access);
            }
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
                    if (reg == PrivReg::CORE_REQ)
                        res |= WROTE_CORE_REQ;
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
                        printEpAccess(lastEp, pkt->isRead(), access);
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
        printEpAccess(lastEp, pkt->isRead(), access);

    if (pkt->needsResponse())
        pkt->makeResponse();

    return static_cast<Result>(res);
}

Addr
RegFile::getSize() const
{
    return TcuTlb::PAGE_SIZE * 2 + sizeof(reg_t) * numPrivRegs;
}
