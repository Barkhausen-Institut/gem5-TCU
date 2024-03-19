/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
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
#include "mem/packet_access.hh"
#include "mem/tcu/reg_file.hh"
#include "mem/tcu/tcu.hh"
#include "sim/tile_memory.hh"

namespace gem5
{
namespace tcu
{

#define DPRINTFNS(name, ...) do {                                       \
    trace::getDebugLogger()->dprintf(curTick(), name, __VA_ARGS__);     \
} while (0)

const char *RegFile::extRegNames[] = {
    "FEATURES",
    "TILE_DESC",
    "EXT_CMD",
    "EPS_ADDR",
    "EPS_SIZE",
};

const char *RegFile::privRegNames[] = {
    "PRIV_CTRL",
    "CU_REQ",
    "PRIV_CMD",
    "PRIV_CMD_ARG",
    "CUR_ACT",
};

const char *RegFile::unprivRegNames[] = {
    "COMMAND",
    "DATA_ADDR",
    "DATA_SIZE",
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
    return (read && debug::TcuRegRead) || (!read && debug::TcuRegWrite);
}

static const char *regAccessName(RegAccess access)
{
    if (access == RegAccess::CPU)
        return "CPU";
    if (access == RegAccess::NOC)
        return "NOC";
    return "TCU";
}

struct RegAccessEvent : public Event
{
    RegFile &_regs;
    PacketPtr _pkt;
    Tcu::RegFileEvent *_regfile_event;
    bool _isCpuRequest;
    int _result;
    std::map<size_t, size_t> _accesses;

    RegAccessEvent(Tcu &tcu, RegFile &regs, Tcu::RegFileEvent *regfile_event,
                   PacketPtr pkt, bool isCpuRequest)
        : Event(),
        _regs(regs),
        _pkt(pkt),
        _regfile_event(regfile_event),
        _isCpuRequest(isCpuRequest),
        _result(),
        _accesses()
    {}

    template<typename F>
    void unaligned_reg_read(size_t offset, F read);
    template<typename R, typename W>
    void unaligned_reg_write(size_t offset, R read, W write);

    void process() override;
    void fetched(EpFile::EpCache &cache);
    void completed(EpFile::EpCache &cache, bool eventScheduled, bool cacheScheduled);
    const std::string name() const override;
    const char* description() const override { return "RegAccessEvent"; }
    // for the address translation (we don't speculate)
    bool isSquashed() const { return false; }
};

RegFile::RegFile(Tcu &_tcu, const std::string& name)
    : tcu(_tcu),
      epsAddr(0),
      extRegs(numExtRegs, 0),
      privRegs(numPrivRegs, 0),
      unprivRegs(numUnprivRegs, 0),
      bufRegs(numBufRegs * sizeof(reg_t), 0),
      _name(name)
{
    static_assert(sizeof(extRegNames) / sizeof(extRegNames[0]) ==
        numExtRegs, "extRegNames out of sync");
    static_assert(sizeof(privRegNames) / sizeof(privRegNames[0]) ==
        numPrivRegs, "privRegNames out of sync");
    static_assert(sizeof(unprivRegNames) / sizeof(unprivRegNames[0]) ==
        numUnprivRegs, "unprivRegNames out of sync");

    // at boot, all tiles are privileged
    FeatureReg feat = 0;
    feat.kernel = 1;
    feat.vm = 0;
    feat.ctxsw = 0;
    feat.vmajor = MAJOR_VERSION;
    feat.vminor = MINOR_VERSION;
    feat.vpatch = PATCH_VERSION;
    set(ExtReg::FEATURES, feat);

    // provide tile description for software
    TileMemory *sys = dynamic_cast<TileMemory*>(tcu.systemObject());
    assert(sys != nullptr);
    set(ExtReg::TILE_DESC, sys->tileDesc(tcu.tileId));

    // set initial EPs; external EPs have a valid NoC address stored in EPS_ADDR
    if (NocAddr(sys->initEpsAddr).valid)
    {
        EPsAddrReg addr = 0;
        addr.tile = NocAddr(sys->initEpsAddr).tileId.raw();
        addr.offset = NocAddr(sys->initEpsAddr).offset;
        set(ExtReg::EPS_ADDR, addr);
    }
    // the TCU has builtin EPs if initEpsAddr is non-zero, but an invalid NoC address
    else if (sys->initEpsAddr != 0)
        epsAddr = sys->initEpsAddr;
    set(ExtReg::EPS_SIZE, sys->initEpsNum * numEpRegs * sizeof(reg_t));

    reset(false);
}

void
RegFile::reset(bool inval)
{
    if(inval)
    {
        // reset all unprivileged and privileged registers
        for(size_t i = 0; i < numUnprivRegs; ++i)
            set(static_cast<UnprivReg>(i), 0);
        for(size_t i = 0; i < numPrivRegs; ++i) {
            if(static_cast<PrivReg>(i) != PrivReg::CUR_ACT)
                set(static_cast<PrivReg>(i), 0);
        }
    }

    // no activity is running (the id might stay invalid for tiles that don't
    // support multiple activities though)
    ActState act = 0;
    act.id = Tcu::INVALID_ACT_ID;
    act.msgs = 0;
    set(PrivReg::CUR_ACT, act);
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
RegFile::printEpAccess(const Named &obj, const Ep &ep, bool read, RegAccess access)
{
    if(!isTraceEnabled(read))
        return;

    switch (ep.type())
    {
        case EpType::INVALID:
            ep.inval.print(obj, false, access);
            break;
        case EpType::SEND:
            ep.send.print(obj, false, access);
            break;
        case EpType::RECEIVE:
            ep.recv.print(obj, false, access);
            break;
        case EpType::MEMORY:
            ep.mem.print(obj, false, access);
            break;
    }
}

void
SendEp::print(const Named &obj,
              bool read,
              RegAccess access) const
{
    DPRINTFNS(obj.name(),
        "%s%s EP%-3u%12s: Send[act=%u, tile=%u ep=%u crdep=%u maxcrd=%u "
                              "curcrd=%u max=%#x lbl=%#llx rpl=%d]\n",
        regAccessName(access), read ? "<-" : "->",
        id, "", r0.act,
        r1.tgtTile, r1.tgtEp, r0.crdEp,
        r0.maxCrd, r0.curCrd, 1 << r0.msgSize,
        r2.label, r0.reply);
}

void
RecvEp::print(const Named &obj,
              bool read,
              RegAccess access) const
{
    DPRINTFNS(obj.name(),
        "%s%s EP%-3u%12s: Recv[act=%u, buf=%p msz=%#x bsz=%#x rpl=%u msgs=%u "
                              "occ=%#018x unr=%#018x rd=%u wr=%u]\n",
        regAccessName(access), read ? "<-" : "->",
        id, "", r0.act,
        r1.buffer, 1 << r0.slotSize, 1 << r0.slots, r0.rplEps,
        unreadMsgs(), r2.occupied, r3.unread, r0.rpos, r0.wpos);
}

void
MemEp::print(const Named &obj,
             bool read,
             RegAccess access) const
{
    DPRINTFNS(obj.name(),
        "%s%s EP%-3u%12s: Mem[act=%u, tile=%u addr=%#llx "
                             "size=%#llx flags=%#x]\n",
        regAccessName(access), read ? "<-" : "->",
        id, "", r0.act, r0.targetTile,
        r1.remoteAddr, r2.remoteSize,
        r0.flags);
}

void
InvalidEp::print(const Named &obj,
                 bool read,
                 RegAccess access) const
{
    DPRINTFNS(obj.name(),
        "%s%s EP%-3u%12s: INVALID (%#x)\n",
        regAccessName(access), read ? "<-" : "->",
        id, "",
        static_cast<unsigned>(type()));
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

const std::string
RegAccessEvent::name() const
{
    return _regfile_event->name();
}

template<typename R>
void RegAccessEvent::unaligned_reg_read(size_t offset, R read)
{
    auto val = read();

    if (_pkt->getSize() == 4)
    {
        uint32_t nval = _pkt->getAddr() % 8 == 0 ? val : (val >> 32);
        _pkt->setLE(nval);
    }
    else if (_pkt->getSize() >= 8 && (_pkt->getSize() % 8) == 0)
        _pkt->getPtr<uint64_t>()[offset / sizeof(RegFile::reg_t)] = val;
    else
    {
        warn("Unsupported register read @ %#lx with %u bytes",
             _pkt->getAddr(), _pkt->getSize());
    }
}

template<typename R, typename W>
void RegAccessEvent::unaligned_reg_write(size_t offset, R read, W write)
{
    RegFile::reg_t val;
    if (_pkt->getSize() == 4)
    {
        val = read();
        uint32_t nval = _pkt->getLE<uint32_t>();
        if (_pkt->getAddr() % 8 == 0)
            val = (val & 0xFFFF'FFFF'0000'0000) | nval;
        else
            val = (val & 0xFFFF'FFFF) | (static_cast<RegFile::reg_t>(nval) << 32);
    }
    else if (_pkt->getSize() >= 8 && (_pkt->getSize() % 8) == 0)
        val = _pkt->getPtr<uint64_t>()[offset / sizeof(RegFile::reg_t)];
    else
    {
        fatal("Unsupported register write @ %#lx with %u bytes",
              _pkt->getAddr(), _pkt->getSize());
    }

    write(val);
}

void
RegAccessEvent::process()
{
    RegAccess access = _isCpuRequest ? RegAccess::CPU : RegAccess::NOC;
    auto *eps = new EpFile::EpCache(_regs.tcu.eps().newCache(EpFile::Regs, "regAccess"));

    // only 8 and 4 byte accesses are supported
    if (_pkt->getAddr() % 8 != 0 && _pkt->getAddr() % 4 != 0)
    {
        warn("Unsupported register access @ %#lx with %u bytes",
             _pkt->getAddr(), _pkt->getSize());
        completed(*eps, true, false);
        return;
    }

    _result = RegFile::WROTE_NONE;

    // perform a single register access for each requested register
    for (unsigned offset = 0; offset < _pkt->getSize(); offset += sizeof(RegFile::reg_t))
    {
        Addr regAddr = _pkt->getAddr() + offset;

        // external registers
        if (regAddr < sizeof(RegFile::reg_t) * numExtRegs)
        {
            auto reg = static_cast<ExtReg>(regAddr / sizeof(RegFile::reg_t));

            if (_pkt->isRead())
                unaligned_reg_read(offset, [this, reg, access] { return _regs.get(reg, access); });
            // external registers can't be set by the CPU
            else if (_pkt->isWrite() && !_isCpuRequest)
            {
                if (reg == ExtReg::EXT_CMD)
                    _result |= RegFile::WROTE_EXT_CMD;
                // EPS_ADDR is always 0 if we have internal EPs
                else if (reg == ExtReg::EPS_ADDR && _regs.epsAddr != 0)
                    continue;
                // TILE_DESC is read-only
                else if (reg == ExtReg::TILE_DESC)
                    continue;

                unaligned_reg_write(
                    offset,
                    [this, reg, access] {
                        return _regs.get(reg, access);
                    },
                    [this, reg, access](RegFile::reg_t val) {
                        _regs.set(reg, val, access);
                    }
                );
            }
        }
        // unprivileged register
        else if (regAddr < sizeof(RegFile::reg_t) * (numExtRegs + numUnprivRegs))
        {
            size_t idx = regAddr / sizeof(RegFile::reg_t) - numExtRegs;
            auto reg = static_cast<UnprivReg>(idx);

            if (_pkt->isRead())
                unaligned_reg_read(offset, [this, reg, access] { return _regs.get(reg, access); });
            // the command registers can't be written from the NoC
            else if (_pkt->isWrite() && _isCpuRequest)
            {
                if(reg == UnprivReg::COMMAND)
                    _result |= RegFile::WROTE_CMD;
                else if(reg == UnprivReg::PRINT)
                    _result |= RegFile::WROTE_PRINT;

                unaligned_reg_write(
                    offset,
                    [this, reg, access] {
                        return _regs.get(reg, access);
                    },
                    [this, reg, access](RegFile::reg_t val) {
                        _regs.set(reg, val, access);
                    }
                );
            }
        }
        // buffer register
        else if (regAddr < sizeof(RegFile::reg_t) * (numExtRegs + numUnprivRegs + numBufRegs))
        {
            Addr bufAddr = regAddr - sizeof(RegFile::reg_t) * (numExtRegs + numUnprivRegs);
            size_t idx = bufAddr / sizeof(RegFile::reg_t);

            if (idx < _regs.bufRegs.size())
            {
                if(_pkt->isRead())
                    unaligned_reg_read(offset, [this, idx] { return _regs.bufRegs[idx]; });
                else {
                    unaligned_reg_write(
                        offset,
                        [this, idx] {
                            return _regs.bufRegs[idx];
                        },
                        [this, idx](RegFile::reg_t val) {
                            _regs.bufRegs[idx] = val;
                        }
                    );
                }
            }
        }
        // privileged register
        else if (regAddr >= TcuTlb::PAGE_SIZE && regAddr < TcuTlb::PAGE_SIZE * 2)
        {
            Addr reqAddr = regAddr - TcuTlb::PAGE_SIZE;

            if (reqAddr < sizeof(RegFile::reg_t) * numPrivRegs)
            {
                auto reg = static_cast<PrivReg>(reqAddr / sizeof(RegFile::reg_t));

                if (_pkt->isRead())
                    unaligned_reg_read(offset, [this, reg, access] { return _regs.get(reg, access); });
                else if (_pkt->isWrite())
                {
                    if (reg == PrivReg::CU_REQ)
                        _result |= RegFile::WROTE_CU_REQ;
                    else if (reg == PrivReg::PRIV_CMD)
                        _result |= RegFile::WROTE_PRIV_CMD;

                    unaligned_reg_write(
                        offset,
                        [this, reg, access] {
                            return _regs.get(reg, access);
                        },
                        [this, reg, access](RegFile::reg_t val) {
                            _regs.set(reg, val, access);
                        }
                    );
                }
            }
        }
        // endpoint register
        else if (regAddr >= TcuTlb::PAGE_SIZE * 2)
        {
            Addr epAddr = regAddr - TcuTlb::PAGE_SIZE * 2;

            // endpoint address
            if (epAddr < _regs.endpointSize())
            {
                epid_t epId = epAddr / (sizeof(RegFile::reg_t) * numEpRegs);
                _accesses[epAddr] = offset;
                eps->addEp(epId);
            }
        }
    }

    if (_accesses.size() > 0)
        eps->onFetched(std::bind(&RegAccessEvent::fetched, this, std::placeholders::_1));
    else
        completed(*eps, true, false);
}

void
RegAccessEvent::fetched(EpFile::EpCache &cache)
{
    RegAccess access = _isCpuRequest ? RegAccess::CPU : RegAccess::NOC;
    bool isPriv = _regs.hasFeature(Features::KERNEL);

    cache.setAccess(access);

    for(auto it = _accesses.begin(); it != _accesses.end(); ++it)
    {
        epid_t epId = it->first / (sizeof(RegFile::reg_t) * numEpRegs);
        unsigned regNumber = (it->first / sizeof(RegFile::reg_t)) % numEpRegs;

        auto ep = cache.getEp(epId);

        if (_pkt->isRead())
            unaligned_reg_read(it->second, [ep, regNumber] { return ep.inval.r[regNumber]; });
        // writable only from remote and privileged TCUs
        else if (!_isCpuRequest || isPriv)
        {
            unaligned_reg_write(
                it->second,
                [ep, regNumber] {
                    return ep.inval.r[regNumber];
                },
                [&ep, regNumber](RegFile::reg_t val) {
                    ep.inval.r[regNumber] = val;
                }
            );
            cache.updateEp(ep.inval);
        }
        else
            assert(false);
    }

    if (_pkt->isRead())
        completed(cache, false, true);
    else
        cache.onFinished(std::bind(&RegAccessEvent::completed, this, std::placeholders::_1, false, true));
}

void
RegAccessEvent::completed(EpFile::EpCache &cache, bool eventScheduled, bool cacheScheduled)
{
    if (_pkt->needsResponse())
        _pkt->makeResponse();

    _regfile_event->completed(static_cast<RegFile::Result>(_result));

    cache.setAutoFinish(true);
    if (cacheScheduled)
        cache.setAutoDelete(true);
    else
        delete &cache;
    if (eventScheduled)
        setFlags(AutoDelete);
    else
        delete this;
}

void
RegFile::startRequest(PacketPtr pkt, void *ev, bool isCpuRequest)
{
    assert(pkt->isRead() || pkt->isWrite());

    auto regfile_event = reinterpret_cast<Tcu::RegFileEvent*>(ev);
    Addr pktAddr = pkt->getAddr();

    DPRINTF(TcuRegRange, "access @%#x, size=%u\n", pktAddr, pkt->getSize());

    auto access_event = new RegAccessEvent(tcu, *this, regfile_event, pkt, isCpuRequest);
    tcu.schedule(access_event, curTick());
}

Addr
RegFile::getSize() const
{
    return TcuTlb::PAGE_SIZE * 2 + sizeof(reg_t) * numEpRegs * tcu.numEndpoints();
}

}
}
