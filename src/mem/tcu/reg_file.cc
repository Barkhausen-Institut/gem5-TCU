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
#include "mem/tcu/reg_file.hh"
#include "mem/tcu/tcu.hh"
#include "sim/tile_memory.hh"

#define DPRINTFNS(name, ...) do {                                       \
    Trace::getDebugLogger()->dprintf(curTick(), name, __VA_ARGS__);     \
} while (0)

const char *RegFile::extRegNames[] = {
    "FEATURES",
    "EXT_CMD",
};

const char *RegFile::privRegNames[] = {
    "CORE_REQ",
    "PRIV_CMD",
    "PRIV_CMD_ARG",
    "CUR_ACT",
    "CLEAR_IRQ",
};

const char *RegFile::unprivRegNames[] = {
    "COMMAND",
    "DATA",
    "ARG1",
    "CUR_TIME",
    "PRINT",
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

RegFile::RegFile(Tcu &_tcu, const std::string& name, unsigned numEndpoints)
    : tcu(_tcu),
      extRegs(numExtRegs, 0),
      privRegs(numPrivRegs, 0),
      unprivRegs(numUnprivRegs, 0),
      eps(),
      bufRegs(numBufRegs * sizeof(reg_t), 0),
      _name(name)
{
    static_assert(sizeof(extRegNames) / sizeof(extRegNames[0]) ==
        numExtRegs, "extRegNames out of sync");
    static_assert(sizeof(privRegNames) / sizeof(privRegNames[0]) ==
        numPrivRegs, "privRegNames out of sync");
    static_assert(sizeof(unprivRegNames) / sizeof(unprivRegNames[0]) ==
        numUnprivRegs, "unprivRegNames out of sync");

    for(unsigned i = 0; i < numEndpoints; ++i)
        eps.push_back(Ep(i));

    // at boot, all tiles are privileged
    reg_t feat = static_cast<reg_t>(Features::PRIV);
    set(ExtReg::FEATURES, feat);

    // and no activity is running (the id might stay invalid for tiles that don't
    // support multiple activities though)
    ActState act = 0;
    act.id = Tcu::INVALID_ACT_ID;
    act.msgs = 0;
    set(PrivReg::CUR_ACT, act);

    initMemEp();
}

void
RegFile::initMemEp()
{
    TileMemory *sys = dynamic_cast<TileMemory*>(tcu.systemObject());
    if (sys)
    {
        NocAddr phys = sys->getPhys(0);

        MemEp ep;
        ep.r0.type = static_cast<RegFile::reg_t>(EpType::MEMORY);
        ep.r0.act = Tcu::INVALID_ACT_ID;
        // TODO exec
        ep.r0.flags = Tcu::MemoryFlags::READ | Tcu::MemoryFlags::WRITE;
        ep.r0.targetTile = phys.tileId;
        ep.r1.remoteAddr = phys.offset;
        ep.r2.remoteSize = sys->memSize;

        updateEp(ep);
    }
}

void
RegFile::add_msg()
{
    ActState cur = getAct(PrivReg::CUR_ACT);
    cur.msgs = cur.msgs + 1;
    set(PrivReg::CUR_ACT, cur);
}

void
RegFile::rem_msg()
{
    ActState cur = getAct(PrivReg::CUR_ACT);
    cur.msgs = cur.msgs - 1;
    set(PrivReg::CUR_ACT, cur);
}

RegFile::reg_t
RegFile::get(ExtReg reg, RegAccess access) const
{
    reg_t value = extRegs[static_cast<Addr>(reg)];

    DPRINTF(TcuRegRead, "%s<- TCU[%-12s]: %#018x\n",
                        regAccessName(access),
                        extRegNames[static_cast<Addr>(reg)],
                        value);

    return value;
}

void
RegFile::set(ExtReg reg, reg_t value, RegAccess access)
{
    DPRINTF(TcuRegWrite, "%s-> TCU[%-12s]: %#018x\n",
                         regAccessName(access),
                         extRegNames[static_cast<Addr>(reg)],
                         value);

    extRegs[static_cast<Addr>(reg)] = value;
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
RegFile::get(UnprivReg reg, RegAccess access) const
{
    reg_t value;

    if (reg == UnprivReg::CUR_TIME)
        value = curTick() / 1000;
    else
        value = unprivRegs[static_cast<Addr>(reg)];

    DPRINTF(TcuRegRead, "%s<- CMD[%-12s]: %#018x\n",
                        regAccessName(access),
                        unprivRegNames[static_cast<Addr>(reg)],
                        value);

    return value;
}

void
RegFile::set(UnprivReg reg, reg_t value, RegAccess access)
{
    DPRINTF(TcuRegWrite, "%s-> CMD[%-12s]: %#018x\n",
                         regAccessName(access),
                         unprivRegNames[static_cast<Addr>(reg)],
                         value);

    unprivRegs[static_cast<Addr>(reg)] = value;
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
            ep.inval.print(*this, false, access);
            break;
        case EpType::SEND:
            ep.send.print(*this, false, access);
            break;
        case EpType::RECEIVE:
            ep.recv.print(*this, false, access);
            break;
        case EpType::MEMORY:
            ep.mem.print(*this, false, access);
            break;
    }
}

void
SendEp::print(const RegFile &rf,
              bool read,
              RegAccess access) const
{
    DPRINTFNS(rf.name(),
        "%s%s EP%-3u%12s: Send[act=%u, tile=%u ep=%u crdep=%u maxcrd=%u "
                              "curcrd=%u max=%#x lbl=%#llx rpl=%d]\n",
        regAccessName(access), read ? "<-" : "->",
        id, "", r0.act,
        r1.tgtTile, r1.tgtEp, r0.crdEp,
        r0.maxCrd, r0.curCrd, 1 << r0.msgSize,
        r2.label, r0.reply);
}

void
RecvEp::print(const RegFile &rf,
              bool read,
              RegAccess access) const
{
    DPRINTFNS(rf.name(),
        "%s%s EP%-3u%12s: Recv[act=%u, buf=%p msz=%#x bsz=%#x rpl=%u msgs=%u "
                              "occ=%#010x unr=%#010x rd=%u wr=%u]\n",
        regAccessName(access), read ? "<-" : "->",
        id, "", r0.act,
        r1.buffer, 1 << r0.slotSize, 1 << r0.slots, r0.rplEps,
        unreadMsgs(), r2.occupied, r2.unread, r0.rpos, r0.wpos);
}

void
MemEp::print(const RegFile &rf,
             bool read,
             RegAccess access) const
{
    DPRINTFNS(rf.name(),
        "%s%s EP%-3u%12s: Mem[act=%u, tile=%u addr=%#llx "
                             "size=%#llx flags=%#x]\n",
        regAccessName(access), read ? "<-" : "->",
        id, "", r0.act, r0.targetTile,
        r1.remoteAddr, r2.remoteSize,
        r0.flags);
}

void
InvalidEp::print(const RegFile &rf,
                 bool read,
                 RegAccess access) const
{
    DPRINTFNS(rf.name(),
        "%s%s EP%-3u%12s: INVALID (%#x)\n",
        regAccessName(access), read ? "<-" : "->",
        id, "",
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

        // external registers
        if (regAddr < sizeof(reg_t) * numExtRegs)
        {
            auto reg = static_cast<ExtReg>(regAddr / sizeof(reg_t));

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg, access);
            // external registers can't be set by the CPU
            else if (pkt->isWrite() && !isCpuRequest)
            {
                if (reg == ExtReg::EXT_CMD)
                    res |= WROTE_EXT_CMD;
                set(reg, data[offset / sizeof(reg_t)], access);
            }
        }
        else if (regAddr >= TcuTlb::PAGE_SIZE * 2)
        {
            Addr reqAddr = regAddr - TcuTlb::PAGE_SIZE * 2;

            if (reqAddr < sizeof(reg_t) * numPrivRegs)
            {
                auto reg = static_cast<PrivReg>(reqAddr / sizeof(reg_t));

                if (pkt->isRead())
                    data[offset / sizeof(reg_t)] = get(reg, access);
                else if (pkt->isWrite())
                {
                    if (reg == PrivReg::CORE_REQ)
                        res |= WROTE_CORE_REQ;
                    else if (reg == PrivReg::PRIV_CMD)
                        res |= WROTE_PRIV_CMD;
                    else if (reg == PrivReg::CLEAR_IRQ)
                        res |= WROTE_CLEAR_IRQ;
                    set(reg, data[offset / sizeof(reg_t)], access);
                }
            }
        }
        // unprivileged register
        else if (regAddr < sizeof(reg_t) * (numExtRegs + numUnprivRegs))
        {
            size_t idx = regAddr / sizeof(reg_t) - numExtRegs;
            auto reg = static_cast<UnprivReg>(idx);

            if (pkt->isRead())
                data[offset / sizeof(reg_t)] = get(reg, access);
            // the command registers can't be written from the NoC
            else if (pkt->isWrite() && isCpuRequest)
            {
                if(reg == UnprivReg::COMMAND)
                    res |= WROTE_CMD;
                else if(reg == UnprivReg::PRINT)
                    res |= WROTE_PRINT;
                set(reg, data[offset / sizeof(reg_t)], access);
            }
        }
        else
        {
            size_t nonEpRegs = numExtRegs + numUnprivRegs;
            Addr epAddr = regAddr - sizeof(reg_t) * nonEpRegs;

            // endpoint address
            if (epAddr < sizeof(reg_t) * numEpRegs * eps.size())
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
                // writable only from remote and privileged TCUs
                else if (!isCpuRequest || isPriv)
                    set(epId, regNumber, data[offset / sizeof(reg_t)]);
                else
                    assert(false);
            }
            // buf register
            else
            {
                Addr bufAddr = epAddr - sizeof(reg_t) * numEpRegs * eps.size();
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
