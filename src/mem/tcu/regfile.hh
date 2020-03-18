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

#ifndef __MEM_TCU_REGFILE_HH__
#define __MEM_TCU_REGFILE_HH__

#include <vector>

#include "base/types.hh"
#include "base/bitfield.hh"
#include "mem/tcu/error.hh"
#include "mem/packet.hh"

// only writable by remote TCUs
enum class TcuReg : Addr
{
    FEATURES,
    CUR_TIME,
    CLEAR_IRQ,
    CLOCK,
};

enum class Features
{
    PRIV            = 1 << 0,
};

constexpr unsigned numTcuRegs = 4;

// privileged registers (for kernel and PEMux)
enum class PrivReg : Addr
{
    CORE_REQ,
    CORE_RESP,
    PRIV_CMD,
    EXT_CMD,
    // MSGS[16] | VPE_ID[16] | EVENTS[3]
    CUR_VPE,
    OLD_VPE,
};

constexpr unsigned numPrivRegs = 6;

// registers to issue a command
enum class CmdReg : Addr
{
    COMMAND,
    ABORT,
    DATA,
    ARG1,
};

constexpr unsigned numCmdRegs = 4;

// Ep Registers:
//
// 0. VPEID[16] | TYPE[3] (for all)
//    receive: BUF_RD_POS[6] | BUF_WR_POS[6] | BUF_MSG_SIZE[6] | BUF_SIZE[6] | REPLY_EPS[16]
//    send:    FLAGS[2] | CRD_EP[16] | MAX_MSG_SIZE[6] | MAXCRD[6] | CURCRD[6]
//    mem:     REQ_PEID[8] | FLAGS[4]
// 1. receive: BUF_ADDR[32]
//    send:    TGT_PEID[8] | TGT_EPID[16]
//    mem:     REQ_MEM_ADDR[64]
// 2. receive: BUF_UNREAD[32] | BUF_OCCUPIED[32]
//    send:    LABEL[32]
//    mem:     REQ_MEM_SIZE[64]
//
constexpr unsigned numEpRegs = 3;

// buffer for prints (32 * 8 bytes)
constexpr unsigned numBufRegs = 32;

typedef uint16_t vpeid_t;
typedef uint16_t epid_t;
typedef uint8_t peid_t;

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

enum class EventType
{
    CRD_RECV,
    EP_INVAL,
    USER,
};

class RegFile;

struct Ep
{
    Ep() : vpe() {}

    uint16_t vpe;
};

struct SendEp : public Ep
{
    static const uint8_t FL_REPLY   = 1;

    SendEp() : Ep(), flags(), targetPe(), targetEp(), crdEp(), maxcrd(), curcrd(),
               maxMsgSize(), label()
    {}

    void print(const RegFile &rf,
               epid_t epId,
               bool read,
               RegAccess access) const;

    uint8_t flags;
    uint8_t targetPe;
    uint16_t targetEp;
    uint16_t crdEp;
    uint8_t maxcrd;
    uint8_t curcrd;
    uint8_t maxMsgSize;
    uint32_t label;
};

struct RecvEp : public Ep
{
    static const size_t MAX_MSGS    = 32;

    RecvEp() : Ep(), rdPos(), wrPos(), bufAddr(), msgSize(), size(), replyEps(),
               occupied(), unread()
    {}

    int unreadMsgs() const
    {
        return popCount(unread);
    }

    int msgToIdx(Addr msg) const
    {
        if (msgSize == 0)
            return MAX_MSGS;
        int idx = (msg - bufAddr) >> msgSize;
        return (idx >= 0 && idx < MAX_MSGS) ? idx : MAX_MSGS;
    }

    bool isUnread(int idx) const
    {
        return unread & (static_cast<uint32_t>(1) << idx);
    }
    void setUnread(int idx, bool unr)
    {
        if (unr)
            unread |= static_cast<uint32_t>(1) << idx;
        else
            unread &= ~(static_cast<uint32_t>(1) << idx);
    }

    bool isOccupied(int idx) const
    {
        return occupied & (static_cast<uint32_t>(1) << idx);
    }
    void setOccupied(int idx, bool occ)
    {
        if (occ)
            occupied |= static_cast<uint32_t>(1) << idx;
        else
            occupied &= ~(static_cast<uint32_t>(1) << idx);
    }

    void print(const RegFile &rf,
               epid_t epId,
               bool read,
               RegAccess access) const;

    uint8_t rdPos;
    uint8_t wrPos;
    uint32_t bufAddr;
    uint16_t msgSize;
    uint16_t size;
    uint16_t replyEps;
    uint32_t occupied;
    uint32_t unread;
};

struct MemEp : public Ep
{
    MemEp() : Ep(), remoteAddr(), remoteSize(), targetPe(), flags()
    {}

    void print(const RegFile &rf,
               epid_t epId,
               bool read,
               RegAccess access) const;

    uint64_t remoteAddr;
    uint64_t remoteSize;
    uint8_t targetPe;
    uint8_t flags;
};

struct DataReg
{
    DataReg() : addr(), size()
    {}
    DataReg(uint64_t value) : addr(value), size(value >> 32)
    {}
    DataReg(uint32_t _addr, uint32_t _size) : addr(_addr), size(_size)
    {}

    uint64_t value() const
    {
        return addr | (static_cast<uint64_t>(size) << 32);
    }

    uint32_t addr;
    uint32_t size;
};

struct MessageHeader
{
    uint8_t flags : 2,
            replySize: 6;
    uint8_t senderPeId;
    uint16_t senderEpId;
    // for a normal message this is the reply epId
    // for a reply this is the enpoint that receives credits
    uint16_t replyEpId;
    uint16_t length;

    // should be large enough for pointers.
    uint32_t replyLabel;
    uint32_t label;
} M5_ATTR_PACKED;

class Tcu;

class RegFile
{
  public:

    using reg_t = uint64_t;

  public:

    enum Result
    {
        WROTE_NONE      = 0,
        WROTE_CMD       = 1,
        WROTE_PRIV_CMD  = 2,
        WROTE_EXT_CMD   = 4,
        WROTE_ABORT     = 8,
        WROTE_XLATE     = 16,
        WROTE_CLEAR_IRQ = 32,
    };

    RegFile(Tcu &tcu, const std::string& name, unsigned numEndpoints);

    bool hasFeature(Features feature) const
    {
        return get(TcuReg::FEATURES) & static_cast<reg_t>(feature);
    }

    TcuError invalidate(epid_t epId, bool force, unsigned *unreadMask);

    void add_msg();

    void rem_msg();

    unsigned messages() const
    {
        return (get(PrivReg::CUR_VPE) >> 3) & 0xFFFF;
    }

    bool hasEvents() const
    {
        return (get(PrivReg::CUR_VPE) & 0x7) != 0;
    }

    reg_t fetchEvents();

    void setEvent(EventType ev);

    reg_t get(TcuReg reg, RegAccess access = RegAccess::TCU) const;

    void set(TcuReg reg, reg_t value, RegAccess access = RegAccess::TCU);

    reg_t get(PrivReg reg, RegAccess access = RegAccess::TCU) const;

    void set(PrivReg reg, reg_t value, RegAccess access = RegAccess::TCU);

    reg_t get(CmdReg reg, RegAccess access = RegAccess::TCU) const;

    void set(CmdReg reg, reg_t value, RegAccess access = RegAccess::TCU);

    reg_t getVPE() const {
        return get(PrivReg::CUR_VPE) >> 19;
    }

    void updateMsgCnt();

    DataReg getDataReg() const
    {
        return DataReg(get(CmdReg::DATA));
    }

    void setDataReg(const DataReg &data)
    {
        set(CmdReg::DATA, data.value());
    }

    SendEp getSendEp(epid_t epId, bool print = true) const;

    void setSendEp(epid_t epId, const SendEp &ep);

    RecvEp getRecvEp(epid_t epId, bool print = true) const;

    void setRecvEp(epid_t epId, const RecvEp &ep);

    MemEp getMemEp(epid_t epId, bool print = true) const;

    const char *getBuffer(size_t bytes);

    /// returns which command registers have been written
    Result handleRequest(PacketPtr pkt, bool isCpuRequest);

    const std::string name() const { return _name; }

  private:

    reg_t get(epid_t epId, size_t idx) const;

    void set(epid_t epId, size_t idx, reg_t value);

    EpType getEpType(epid_t epId) const;

    void printEpAccess(epid_t epId, bool read, bool cpu) const;

    Addr getSize() const;

    unsigned countMsgs(vpeid_t vpeId);

  private:

    Tcu &tcu;

    std::vector<reg_t> tcuRegs;

    std::vector<reg_t> privRegs;

    std::vector<reg_t> cmdRegs;

    std::vector<std::vector<reg_t>> epRegs;

    std::vector<reg_t> bufRegs;

    const unsigned numEndpoints;

    // used for debug messages (DPRINTF)
    const std::string _name;

  private:

    static const char *tcuRegNames[];
    static const char *privRegNames[];
    static const char *cmdRegNames[];
    static const char *epTypeNames[];
};

#endif // __MEM_TCU_REGFILE_HH__
