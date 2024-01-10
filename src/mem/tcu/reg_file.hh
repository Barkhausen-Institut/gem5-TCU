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

#ifndef __MEM_TCU_REG_FILE_HH__
#define __MEM_TCU_REG_FILE_HH__

#include <vector>

#include "base/types.hh"
#include "base/bitunion.hh"
#include "mem/tcu/error.hh"
#include "mem/packet.hh"
#include "sim/eventq.hh"

namespace gem5
{

namespace tcu
{

static constexpr Addr PATCH_VERSION = 0;
static constexpr Addr MINOR_VERSION = 0;
static constexpr Addr MAJOR_VERSION = 2;

// external registers (only externally writable)
enum class ExtReg : Addr
{
    FEATURES,
    TILE_DESC,
    EXT_CMD,
    EPS_ADDR,
    EPS_SIZE,
};

enum class Features
{
    KERNEL  = 1 << 0,
    VM      = 1 << 1,
    CTXSW   = 1 << 2,
};

// privileged registers (only writable by privileged software)
enum class PrivReg : Addr
{
    CU_REQ,
    PRIV_CTRL,
    PRIV_CMD,
    PRIV_CMD_ARG1,
    CUR_ACT,
    CLEAR_IRQ,
};

enum PrivCtrl
{
    PMP_FAILURES  = 1 << 0,
};

// unprivileged registers (writable by the application)
enum class UnprivReg : Addr
{
    COMMAND,
    DATA_ADDR,
    DATA_SIZE,
    ARG1,
    CUR_TIME,
    PRINT,
};

constexpr unsigned numExtRegs = 5;
constexpr unsigned numPrivRegs = 6;
constexpr unsigned numUnprivRegs = 6;
constexpr unsigned numEpRegs = 4;
// buffer for prints (32 * 8 bytes)
constexpr unsigned numBufRegs = 32;

typedef uint16_t actid_t;
typedef uint16_t epid_t;
typedef uint8_t chipid_t;
typedef uint8_t tileid_t;
typedef uint64_t tiledesc_t;

class TileId
{
public:
    static TileId from_raw(uint16_t raw)
    {
        return TileId(raw >> 8, raw & 0xFF);
    }

    explicit TileId() : id() {}
    explicit TileId(chipid_t chip, tileid_t tile) : id((chip << 8) | tile) {}

    chipid_t chip() const { return id >> 8; }
    tileid_t tile() const { return id & 0xFF; }
    uint16_t raw() const { return id; }

    bool operator <(const TileId &rhs) const { return id < rhs.id; }
    friend bool operator==(const TileId &lhs, const TileId &rhs)
    {
        return lhs.id == rhs.id;
    }
    friend bool operator!=(const TileId &lhs, const TileId &rhs)
    {
        return !operator==(lhs, rhs);
    }

    friend std::ostream &operator<<(std::ostream &os, const TileId &t)
    {
        ccprintf(os, "C%dT%02d", t.chip(), t.tile());
        return os;
    }

private:
    uint16_t id;
};

/**
 *
 *  64 63      49        0
 *   ---------------------
 *   |V| tileId | offset |
 *   ---------------------
 */
class NocAddr
{
  public:

    explicit NocAddr() : valid(), tileId(), offset()
    {}

    explicit NocAddr(Addr addr)
        : valid(addr >> 63),
          tileId(TileId::from_raw((addr >> 49) & ((1 << 14) - 1))),
          offset(addr & ((static_cast<Addr>(1) << 49) - 1))
    {}

    explicit NocAddr(TileId _tileId, Addr _offset)
        : valid(1), tileId(_tileId), offset(_offset)
    {}

    Addr getAddr() const
    {
        assert((tileId.raw() & ~((1 << 14) - 1)) == 0);
        assert((offset & ~((static_cast<Addr>(1) << 49) - 1)) == 0);

        Addr res = static_cast<Addr>(valid) << 63;
        res |= static_cast<Addr>(tileId.raw()) << 49;
        res |= offset;
        return res;
    }

    bool valid;

    TileId tileId;

    Addr offset;
};

enum class EpType
{
    INVALID,
    SEND,
    RECEIVE,
    MEMORY,
};

enum class RegAccess
{
    CPU,
    TCU,
    NOC
};

struct CmdCommand
{
    enum Opcode
    {
        IDLE            = 0,
        SEND            = 1,
        REPLY           = 2,
        READ            = 3,
        WRITE           = 4,
        FETCH_MSG       = 5,
        ACK_MSG         = 6,
        SLEEP           = 7,
    };

    BitUnion64(Bits)
        Bitfield<57, 25> arg0;
        Bitfield<24, 20> error;
        Bitfield<19, 4> epid;
        Bitfield<3, 0> opcode;
    EndBitUnion(Bits)

    static Bits create(Opcode op, epid_t ep, uint64_t arg0 = 0)
    {
        Bits cmd = 0;
        cmd.opcode = op;
        cmd.epid = ep;
        cmd.arg0 = arg0;
        return cmd;
    }
};

struct CmdData
{
    uint64_t addr;
    uint64_t size;

    static CmdData create(uint64_t addr, uint64_t size)
    {
        return CmdData {
            .addr = addr,
            .size = size,
        };
    }
};

struct PrivCommand
{
    enum Opcode
    {
        IDLE            = 0,
        INV_PAGE        = 1,
        INV_TLB         = 2,
        INS_TLB         = 3,
        XCHG_ACT        = 4,
        SET_TIMER       = 5,
        ABORT_CMD       = 6,
    };

    BitUnion64(Bits)
        Bitfield<64, 9> arg0;
        Bitfield<8, 4> error;
        Bitfield<3, 0> opcode;
    EndBitUnion(Bits)
};

struct ExtCommand
{
    enum Opcode
    {
        IDLE            = 0,
        INV_EP          = 1,
        RESET           = 2,
    };

    BitUnion64(Bits)
        Bitfield<64, 9> arg;
        Bitfield<8, 4> error;
        Bitfield<3, 0> opcode;
    EndBitUnion(Bits)
};

class RegFile;

struct SendEp
{
    static const uint8_t FL_REPLY   = 1;

    explicit SendEp() : id(), r0(0), r1(0), r2(0), r3(0) {}

    void print(const Named &obj,
               bool read,
               RegAccess access) const;

    BitUnion64(R0)
        Bitfield<55> reply;
        Bitfield<54, 39> crdEp;
        Bitfield<38, 33> msgSize;
        Bitfield<32, 26> maxCrd;
        Bitfield<25, 19> curCrd;
        Bitfield<18, 3> act;
        Bitfield<2, 0> type;
    EndBitUnion(R0)

    BitUnion64(R1)
        Bitfield<29, 16> tgtTile;
        Bitfield<15, 0> tgtEp;
    EndBitUnion(R1)

    BitUnion64(R2)
        Bitfield<63, 0> label;
    EndBitUnion(R2)

    epid_t id;
    R0 r0;
    R1 r1;
    R2 r2;
    uint64_t r3;
};

struct RecvEp
{
    static const size_t MAX_MSGS    = 64;

    explicit RecvEp() : id(), r0(0), r1(0), r2(0), r3(0) {}

    int unreadMsgs() const
    {
        return popCount(r3.unread);
    }

    int offsetToIdx(Addr off) const
    {
        if (r0.slotSize == 0)
            return MAX_MSGS;
        int idx = off >> r0.slotSize;
        return (idx >= 0 && idx < (1 << r0.slots)) ? idx : MAX_MSGS;
    }

    bool isUnread(int idx) const
    {
        return r3.unread & (static_cast<uint64_t>(1) << idx);
    }
    void setUnread(int idx, bool unr)
    {
        if (unr)
            r3.unread = r3.unread | (static_cast<uint64_t>(1) << idx);
        else
            r3.unread = r3.unread & ~(static_cast<uint64_t>(1) << idx);
    }

    bool isOccupied(int idx) const
    {
        return r2.occupied & (static_cast<uint64_t>(1) << idx);
    }
    void setOccupied(int idx, bool occ)
    {
        if (occ)
            r2.occupied = r2.occupied | (static_cast<uint64_t>(1) << idx);
        else
            r2.occupied = r2.occupied & ~(static_cast<uint64_t>(1) << idx);
    }

    void print(const Named &obj,
               bool read,
               RegAccess access) const;

    BitUnion64(R0)
        Bitfield<61, 55> rpos;
        Bitfield<54, 48> wpos;
        Bitfield<47, 42> slotSize;
        Bitfield<41, 35> slots;
        Bitfield<34, 19> rplEps;
        Bitfield<18, 3> act;
        Bitfield<2, 0> type;
    EndBitUnion(R0)

    BitUnion64(R1)
        Bitfield<63, 0> buffer;
    EndBitUnion(R1)

    BitUnion64(R2)
        Bitfield<63, 0> occupied;
    EndBitUnion(R2)

    BitUnion64(R3)
        Bitfield<63, 0> unread;
    EndBitUnion(R3)

    epid_t id;
    R0 r0;
    R1 r1;
    R2 r2;
    R3 r3;
};

struct MemEp
{
    explicit MemEp() : id(), r0(0), r1(0), r2(0), r3(0) {}

    void print(const Named &obj,
               bool read,
               RegAccess access) const;

    BitUnion64(R0)
        Bitfield<36, 23> targetTile;
        Bitfield<22, 19> flags;
        Bitfield<18, 3> act;
        Bitfield<2, 0> type;
    EndBitUnion(R0)

    BitUnion64(R1)
        Bitfield<63, 0> remoteAddr;
    EndBitUnion(R1)

    BitUnion64(R2)
        Bitfield<63, 0> remoteSize;
    EndBitUnion(R2)

    epid_t id;
    R0 r0;
    R1 r1;
    R2 r2;
    uint64_t r3;
};

struct InvalidEp
{
    void print(const Named &obj,
               bool read,
               RegAccess access) const;

    EpType type() const
    {
        return static_cast<EpType>(r[0] & 0x7);
    }

    epid_t id;
    union {
        uint64_t r[4];
        struct {
            uint64_t r0;
            uint64_t r1;
            uint64_t r2;
            uint64_t r3;
        };
    };
};

union Ep
{
    explicit Ep() : Ep(0xFFFF) {}
    explicit Ep(epid_t id) : inval({id, {0, 0, 0, 0}}) {}

    Ep(const Ep &ep)
    {
        inval.id = ep.inval.id;
        for(size_t i = 0; i < numEpRegs; ++i)
            inval.r[i] = ep.inval.r[i];
    }
    Ep &operator=(const Ep &ep)
    {
        if(&ep != this)
            inval = ep.inval;
        return *this;
    }

    EpType type() const
    {
        return inval.type();
    }

    template<class E>
    void update(E &ep)
    {
        inval.r[0] = ep.r0;
        inval.r[1] = ep.r1;
        inval.r[2] = ep.r2;
        inval.r[3] = ep.r3;
    }

    InvalidEp inval;
    SendEp send;
    RecvEp recv;
    MemEp mem;
};

BitUnion64(ActState)
    Bitfield<31, 16> msgs;
    Bitfield<15, 0> id;
EndBitUnion(ActState)

enum CUMsgType
{
    IDLE = 0,
    RESP = 1,
    FOREIGN_REQ = 2,
    PMP_FAILURE = 3,
};

BitUnion64(CUMsg)
    Bitfield<1, 0> type;
EndBitUnion(CUMsg)

BitUnion64(ForeignCUReq)
    Bitfield<2, 0> type;
    Bitfield<18, 3> ep;
    Bitfield<63, 48> act;
EndBitUnion(ForeignCUReq)

BitUnion64(ForeignCUResp)
    Bitfield<2, 0> type;
EndBitUnion(ForeignCUResp)

BitUnion64(PMPFailureCUReq)
    Bitfield<2, 0> type;
    Bitfield<3, 3> write;
    Bitfield<8, 4> error;
    Bitfield<63, 32> phys;
EndBitUnion(PMPFailureCUReq)

BitUnion64(FeatureReg)
    Bitfield<63, 56> vpatch;
    Bitfield<55, 48> vminor;
    Bitfield<47, 32> vmajor;
    Bitfield<2, 2> ctxsw;
    Bitfield<1, 1> vm;
    Bitfield<0, 0> kernel;
EndBitUnion(FeatureReg)

BitUnion64(EPsAddrReg)
    Bitfield<63, 56> tile;
    Bitfield<55, 0> offset;
EndBitUnion(EPsAddrReg)

BitUnion64(PrintReg)
    Bitfield<23, 0> size;
    Bitfield<55, 24> cov_addr;
    Bitfield<63, 56> cov_act;
EndBitUnion(PrintReg)

struct M5_ATTR_PACKED MessageHeader
{
    uint32_t flags : 1,
             replySize : 4,
             senderTileId : 14,
             length : 13;
    uint16_t senderEpId;
    // for a normal message this is the reply epId
    // for a reply this is the enpoint that receives credits
    uint16_t replyEpId;

    // should be large enough for pointers.
    uint64_t replyLabel;
    uint64_t label;
    // padding
    uint64_t : 64;
};

class Tcu;
class EpFile;
struct RegAccessEvent;

class RegFile
{
    friend class Tcu;
    friend class EpFile;
    friend struct RegAccessEvent;

  public:

    using reg_t = uint64_t;

  public:

    enum Result
    {
        WROTE_NONE      = 0,
        WROTE_CMD       = 1,
        WROTE_PRIV_CMD  = 2,
        WROTE_EXT_CMD   = 4,
        WROTE_CU_REQ    = 8,
        WROTE_CLEAR_IRQ = 16,
        WROTE_PRINT     = 32,
    };

    static void printEpAccess(const Named &obj, const Ep &ep, bool read, RegAccess access);

    RegFile(Tcu &tcu, const std::string& name);

    void reset(bool inval);

    bool hasFeature(Features feature) const
    {
        return get(ExtReg::FEATURES) & static_cast<reg_t>(feature);
    }

    Addr endpointsAddr() const
    {
        if (epsAddr != 0)
            return epsAddr;

        EPsAddrReg reg = get(ExtReg::EPS_ADDR);
        panic_if((reg.offset & 0x1f) != 0, "EPS_ADDR.offset not 32-byte aligned");
        return NocAddr(TileId::from_raw(reg.tile), reg.offset).getAddr();
    }
    size_t endpointSize() const
    {
        reg_t reg = get(ExtReg::EPS_SIZE);
        panic_if((reg & 0x1f) != 0, "EPS_SIZE not 32-byte aligned");
        return reg;
    }

    void add_msg();

    void rem_msg();

    ActState getAct(PrivReg reg) const
    {
        return get(reg);
    }

    ActState getCurAct() const
    {
        return getAct(PrivReg::CUR_ACT);
    }

    reg_t get(ExtReg reg, RegAccess access = RegAccess::TCU) const;

    void set(ExtReg reg, reg_t value, RegAccess access = RegAccess::TCU);

    reg_t get(PrivReg reg, RegAccess access = RegAccess::TCU) const;

    void set(PrivReg reg, reg_t value, RegAccess access = RegAccess::TCU);

    reg_t get(UnprivReg reg, RegAccess access = RegAccess::TCU) const;

    void set(UnprivReg reg, reg_t value, RegAccess access = RegAccess::TCU);

    CmdCommand::Bits getCommand()
    {
        return get(UnprivReg::COMMAND);
    }

    CmdData getData() const
    {
        return CmdData::create(get(UnprivReg::DATA_ADDR), get(UnprivReg::DATA_SIZE));
    }

    void setData(const CmdData &data)
    {
        set(UnprivReg::DATA_ADDR, data.addr);
        set(UnprivReg::DATA_SIZE, data.size);
    }

    const char *getBuffer(size_t bytes);

    void startRequest(PacketPtr pkt, void *ev, bool isCpuRequest);

    const std::string name() const { return _name; }

  private:

    Addr getSize() const;

  private:

    Tcu &tcu;

    Addr epsAddr;

    std::vector<reg_t> extRegs;

    std::vector<reg_t> privRegs;

    std::vector<reg_t> unprivRegs;

    std::vector<reg_t> bufRegs;

    // used for debug messages (DPRINTF)
    const std::string _name;

  private:

    static const char *extRegNames[];
    static const char *privRegNames[];
    static const char *unprivRegNames[];
    static const char *epTypeNames[];
};

}
}

#endif // __MEM_TCU_REG_FILE_HH__
