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

#ifndef __MEM_DTU_REGFILE_HH__
#define __MEM_DTU_REGFILE_HH__

#include <vector>

#include "base/types.hh"
#include "mem/packet.hh"

// only writable by master DTUs
enum class MasterReg : Addr
{
    FEATURES,
    ROOT_PT,
    PF_EP,
    RW_BARRIER,
    VPE_ID,
    CUR_TIME,
    IDLE_TIME,
    MSG_CNT,
    EXT_CMD,
};

enum class Features
{
    MASTER          = 1 << 0,
    PRIV            = 1 << 1,
    PAGEFAULTS      = 1 << 2,
    COM_DISABLED    = 1 << 3,
    IRQ_WAKEUP      = 1 << 4,
};

constexpr unsigned numMasterRegs = 9;

// only writable by privileged DTUs
enum class PrivReg : Addr
{
    MASTER_REQ,
    XLATE_REQ,
    XLATE_RESP,
};

constexpr unsigned numPrivRegs = 3;

// registers to issue a command
enum class CmdReg : Addr
{
    COMMAND,
    ABORT,
    DATA,
    OFFSET,
    REPLY_LABEL,
};

constexpr unsigned numCmdRegs = 5;

// Ep Registers:
//
// 0. TYPE[3] (for all)
//    receive: BUF_RD_POS[6] | BUF_WR_POS[6] | BUF_MSG_SIZE[16] | BUF_SIZE[16] | BUF_MSG_CNT[16]
//    send:    VPE_ID[32] | MAX_MSG_SIZE[16]
//    mem:     REQ_MEM_SIZE[61]
// 1. receive: BUF_ADDR[64]
//    send:    TGT_COREID[8] | TGT_EPID[8] | MAXCRD[16] | CURCRD[16]
//    mem:     REQ_MEM_ADDR[64]
// 2. receive: BUF_UNREAD[32] | BUF_OCCUPIED[32]
//    send:    LABEL[64]
//    mem:     VPE_ID[32] | REQ_COREID[8] | FLAGS[4]
//
constexpr unsigned numEpRegs = 3;

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
    DTU,
    NOC
};

class RegFile;

struct SendEp
{
    SendEp() : vpeId(), targetCore(), targetEp(), maxMsgSize(), maxcrd(),
               curcrd(), label()
    {}

    void print(const RegFile &rf,
               unsigned epId,
               bool read,
               RegAccess access) const;

    uint32_t vpeId;
    uint8_t targetCore;
    uint8_t targetEp;
    uint16_t maxMsgSize;
    uint16_t maxcrd;
    uint16_t curcrd;
    uint64_t label;
};

struct RecvEp
{
    static const size_t MAX_MSGS    = 32;

    RecvEp() : bufAddr(), msgSize(), size(), msgCount(), occupied(), unread()
    {}

    int msgToIdx(Addr msg) const
    {
        if (msgSize == 0)
            return MAX_MSGS;
        int idx = (msg - bufAddr) / msgSize;
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
               unsigned epId,
               bool read,
               RegAccess access) const;

    uint8_t rdPos;
    uint8_t wrPos;
    uint64_t bufAddr;
    uint16_t msgSize;
    uint16_t size;
    uint16_t msgCount;
    uint32_t occupied;
    uint32_t unread;
};

struct MemEp
{
    MemEp() : vpeId(), remoteAddr(), remoteSize(), targetCore(), flags()
    {}

    void print(const RegFile &rf,
               unsigned epId,
               bool read,
               RegAccess access) const;

    uint32_t vpeId;
    uint64_t remoteAddr;
    uint64_t remoteSize;
    uint8_t targetCore;
    uint8_t flags;
};

struct DataReg
{
    DataReg() : addr(), size()
    {}
    DataReg(uint64_t value) : addr(value & 0xFFFFFFFFFFFF), size(value >> 48)
    {}
    DataReg(uint64_t _addr, uint16_t _size) : addr(_addr), size(_size)
    {}

    uint64_t value() const
    {
        return addr | (static_cast<uint64_t>(size) << 48);
    }

    uint64_t addr;
    uint16_t size;
};

class Dtu;

class RegFile
{
  public:

    using reg_t = uint64_t;

  public:

    enum Result
    {
        WROTE_NONE      = 0,
        WROTE_CMD       = 1,
        WROTE_EXT_CMD   = 2,
        WROTE_ABORT     = 4,
        WROTE_XLATE     = 8,
        WROTE_MST_CMD   = 16,
    };

    RegFile(Dtu &dtu, const std::string& name, unsigned numEndpoints);

    bool hasFeature(Features feature) const
    {
        return get(MasterReg::FEATURES) & static_cast<reg_t>(feature);
    }

    bool invalidate(unsigned epId);

    reg_t get(MasterReg reg, RegAccess access = RegAccess::DTU) const;

    void set(MasterReg reg, reg_t value, RegAccess access = RegAccess::DTU);

    reg_t get(PrivReg reg, RegAccess access = RegAccess::DTU) const;

    void set(PrivReg reg, reg_t value, RegAccess access = RegAccess::DTU);

    reg_t get(CmdReg reg, RegAccess access = RegAccess::DTU) const;

    void set(CmdReg reg, reg_t value, RegAccess access = RegAccess::DTU);

    DataReg getDataReg() const
    {
        return DataReg(get(CmdReg::DATA));
    }

    void setDataReg(const DataReg &data)
    {
        set(CmdReg::DATA, data.value());
    }

    SendEp getSendEp(unsigned epId, bool print = true) const;

    void setSendEp(unsigned epId, const SendEp &ep);

    RecvEp getRecvEp(unsigned epId, bool print = true) const;

    void setRecvEp(unsigned epId, const RecvEp &ep);

    MemEp getMemEp(unsigned epId, bool print = true) const;

    /// returns which command registers have been written
    Result handleRequest(PacketPtr pkt, bool isCpuRequest);

    const std::string name() const { return _name; }

  private:

    reg_t get(unsigned epId, size_t idx) const;

    void set(unsigned epId, size_t idx, reg_t value);

    EpType getEpType(unsigned epId) const;

    void printEpAccess(unsigned epId, bool read, bool cpu) const;

    Addr getSize() const;

  private:

    Dtu &dtu;

    std::vector<reg_t> masterRegs;

    std::vector<reg_t> privRegs;

    std::vector<reg_t> cmdRegs;

    std::vector<std::vector<reg_t>> epRegs;

    const unsigned numEndpoints;

    // used for debug messages (DPRINTF)
    const std::string _name;

  private:

    static const char *masterRegNames[];
    static const char *privRegNames[];
    static const char *cmdRegNames[];
    static const char *epTypeNames[];
};

#endif // __MEM_DTU_REGFILE_HH__
