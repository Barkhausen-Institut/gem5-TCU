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

// external registers (only externally writable)
enum class ExtReg : Addr
{
    FEATURES,
    EXT_CMD,
};

enum class Features
{
    PRIV            = 1 << 0,
};

// privileged registers (only writable by privileged software)
enum class PrivReg : Addr
{
    CORE_REQ,
    PRIV_CMD,
    PRIV_CMD_ARG1,
    CUR_ACT,
    CLEAR_IRQ,
};

// unprivileged registers (writable by the application)
enum class UnprivReg : Addr
{
    COMMAND,
    DATA,
    ARG1,
    CUR_TIME,
    PRINT,
};

constexpr unsigned numExtRegs = 2;
constexpr unsigned numPrivRegs = 5;
constexpr unsigned numUnprivRegs = 5;
constexpr unsigned numEpRegs = 3;
// buffer for prints (32 * 8 bytes)
constexpr unsigned numBufRegs = 32;

typedef uint16_t actid_t;
typedef uint16_t epid_t;
typedef uint8_t tileid_t;

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
    BitUnion64(Bits)
        Bitfield<63, 32> size;
        Bitfield<31, 0> addr;
    EndBitUnion(Bits)

    static Bits create(uint32_t addr, uint32_t size)
    {
        Bits data = 0;
        data.addr = addr;
        data.size = size;
        return data;
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
        FLUSH_CACHE     = 7,
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

    explicit SendEp() : id(), r0(0), r1(0), r2(0) {}

    void print(const RegFile &rf,
               bool read,
               RegAccess access) const;

    BitUnion64(R0)
        Bitfield<53> reply;
        Bitfield<52, 37> crdEp;
        Bitfield<36, 31> msgSize;
        Bitfield<30, 25> maxCrd;
        Bitfield<24, 19> curCrd;
        Bitfield<18, 3> act;
        Bitfield<2, 0> type;
    EndBitUnion(R0)

    BitUnion64(R1)
        Bitfield<23, 16> tgtTile;
        Bitfield<15, 0> tgtEp;
    EndBitUnion(R1)

    BitUnion64(R2)
        Bitfield<31, 0> label;
    EndBitUnion(R2)

    epid_t id;
    R0 r0;
    R1 r1;
    R2 r2;
};

struct RecvEp
{
    static const size_t MAX_MSGS    = 32;

    explicit RecvEp() : id(), r0(0), r1(0), r2(0) {}

    int unreadMsgs() const
    {
        return popCount(r2.unread);
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
        return r2.unread & (static_cast<uint32_t>(1) << idx);
    }
    void setUnread(int idx, bool unr)
    {
        if (unr)
            r2.unread = r2.unread | (static_cast<uint32_t>(1) << idx);
        else
            r2.unread = r2.unread & ~(static_cast<uint32_t>(1) << idx);
    }

    bool isOccupied(int idx) const
    {
        return r2.occupied & (static_cast<uint32_t>(1) << idx);
    }
    void setOccupied(int idx, bool occ)
    {
        if (occ)
            r2.occupied = r2.occupied | (static_cast<uint32_t>(1) << idx);
        else
            r2.occupied = r2.occupied & ~(static_cast<uint32_t>(1) << idx);
    }

    void print(const RegFile &rf,
               bool read,
               RegAccess access) const;

    BitUnion64(R0)
        Bitfield<58, 53> rpos;
        Bitfield<52, 47> wpos;
        Bitfield<46, 41> slotSize;
        Bitfield<40, 35> slots;
        Bitfield<34, 19> rplEps;
        Bitfield<18, 3> act;
        Bitfield<2, 0> type;
    EndBitUnion(R0)

    BitUnion64(R1)
        Bitfield<63, 0> buffer;
    EndBitUnion(R1)

    BitUnion64(R2)
        Bitfield<63, 32> unread;
        Bitfield<31, 0> occupied;
    EndBitUnion(R2)

    epid_t id;
    R0 r0;
    R1 r1;
    R2 r2;
};

struct MemEp
{
    explicit MemEp() : id(), r0(0), r1(0), r2(0) {}

    void print(const RegFile &rf,
               bool read,
               RegAccess access) const;

    BitUnion64(R0)
        Bitfield<30, 23> targetTile;
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
};

struct InvalidEp
{
    void print(const RegFile &rf,
               bool read,
               RegAccess access) const;

    EpType type() const
    {
        return static_cast<EpType>(r[0] & 0x7);
    }

    epid_t id;
    uint64_t r[3];
};

union Ep
{
    explicit Ep() : Ep(0xFFFF) {}
    explicit Ep(epid_t id) : inval({id, {0, 0, 0}}) {}

    Ep(const Ep &ep) {
        inval.id = ep.inval.id;
        for(size_t i = 0; i < numEpRegs; ++i)
            inval.r[i] = ep.inval.r[i];
    }
    Ep &operator=(const Ep &ep) {
        if(&ep != this) {
            inval = ep.inval;
        }
        return *this;
    }

    EpType type() const
    {
        return inval.type();
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

enum CoreMsgType
{
    IDLE = 0,
    RESP = 1,
    FOREIGN_REQ = 2,
};

BitUnion64(CoreMsg)
    Bitfield<1, 0> type;
EndBitUnion(CoreMsg)

BitUnion64(ForeignCoreReq)
    Bitfield<1, 0> type;
    Bitfield<17, 2> ep;
    Bitfield<63, 48> act;
EndBitUnion(ForeignCoreReq)

BitUnion64(ForeignCoreResp)
    Bitfield<1, 0> type;
EndBitUnion(ForeignCoreResp)

struct M5_ATTR_PACKED MessageHeader
{
    uint8_t flags : 2,
            replySize: 6;
    uint8_t senderTileId;
    uint16_t senderEpId;
    // for a normal message this is the reply epId
    // for a reply this is the enpoint that receives credits
    uint16_t replyEpId;
    uint16_t length;

    // should be large enough for pointers.
    uint32_t replyLabel;
    uint32_t label;
};

class Tcu;
class EpFile;

class RegFile
{
    friend class Tcu;
    friend class EpFile;

  public:

    using reg_t = uint64_t;

  public:

    enum Result
    {
        WROTE_NONE      = 0,
        WROTE_CMD       = 1,
        WROTE_PRIV_CMD  = 2,
        WROTE_EXT_CMD   = 4,
        WROTE_CORE_REQ  = 8,
        WROTE_CLEAR_IRQ = 16,
        WROTE_PRINT     = 32,
    };

    RegFile(Tcu &tcu, const std::string& name, unsigned numEndpoints);

    bool hasFeature(Features feature) const
    {
        return get(ExtReg::FEATURES) & static_cast<reg_t>(feature);
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

    CmdData::Bits getData() const
    {
        return CmdData::Bits(get(UnprivReg::DATA));
    }

    void setData(const CmdData::Bits &data)
    {
        set(UnprivReg::DATA, data);
    }

    const char *getBuffer(size_t bytes);

    /// returns which command registers have been written
    Result handleRequest(PacketPtr pkt, bool isCpuRequest);

    const std::string name() const { return _name; }

  private:

    void initMemEp();

    Ep getEp(epid_t epId) const
    {
        printEpAccess(epId, true, RegAccess::TCU);
        return eps.at(epId);
    }

    template<class T>
    void updateEp(const T &ep)
    {
        Ep &old = eps.at(ep.id);
        old.inval.r[0] = ep.r0;
        old.inval.r[1] = ep.r1;
        old.inval.r[2] = ep.r2;
        printEpAccess(ep.id, false, RegAccess::TCU);
    }

    reg_t get(epid_t epId, size_t idx) const;

    void set(epid_t epId, size_t idx, reg_t value);

    void printEpAccess(epid_t epId, bool read, RegAccess access) const;

    Addr getSize() const;

  private:

    Tcu &tcu;

    std::vector<reg_t> extRegs;

    std::vector<reg_t> privRegs;

    std::vector<reg_t> unprivRegs;

    std::vector<Ep> eps;

    std::vector<reg_t> bufRegs;

    // used for debug messages (DPRINTF)
    const std::string _name;

  private:

    static const char *extRegNames[];
    static const char *privRegNames[];
    static const char *unprivRegNames[];
    static const char *epTypeNames[];
};

#endif // __MEM_TCU_REG_FILE_HH__
