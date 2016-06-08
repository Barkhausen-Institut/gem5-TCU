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

// global and readonly for SW
enum class DtuReg : Addr
{
    STATUS,
    ROOT_PT,
    PF_EP,
    LAST_PF,
    RW_BARRIER,
    VPE_ID,
    MSG_CNT,
    EXT_CMD,
};

enum class Status
{
    PRIV        = 1 << 0,
    PAGEFAULTS  = 1 << 1,
};

constexpr unsigned numDtuRegs = 8;

// registers to issue a command
enum class CmdReg : Addr
{
    COMMAND,
    DATA_ADDR,
    DATA_SIZE,
    OFFSET,
    REPLY_EPID,
    REPLY_LABEL,
};

constexpr unsigned numCmdRegs = 6;

// Ep Registers:
//
// 0. TYPE[3] (for all)
//    receive: BUF_MSG_SIZE[16] | BUF_SIZE[16] | BUF_MSG_CNT[16]
//    send:    VPE_ID[32] | MAX_MSG_SIZE[16]
//    mem:     REQ_MEM_SIZE[61]
// 1. receive: BUF_ADDR[64]
//    send:    TGT_COREID[8] | TGT_EPID[8] | CREDITS[16]
//    mem:     REQ_MEM_ADDR[64]
// 2. receive: BUF_RD_PTR[16] | BUF_WR_PTR[16]
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
    SendEp() : vpeId(), targetCore(), targetEp(), maxMsgSize(), credits(), label()
    {}

    void print(const RegFile &rf,
               unsigned epId,
               bool read,
               RegAccess access) const;

    uint32_t vpeId;
    uint8_t targetCore;
    uint8_t targetEp;
    uint16_t maxMsgSize;
    uint16_t credits;
    uint64_t label;
};

struct RecvEp
{
    RecvEp() : bufAddr(), msgSize(), size(), msgCount(), rdOff(), wrOff()
    {}

    void print(const RegFile &rf,
               unsigned epId,
               bool read,
               RegAccess access) const;

    uint64_t bufAddr;
    uint16_t msgSize;
    uint16_t size;
    uint16_t msgCount;
    uint16_t rdOff;
    uint16_t wrOff;
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
    };

    RegFile(const std::string& name, unsigned numEndpoints);

    reg_t get(DtuReg reg, RegAccess access = RegAccess::DTU) const;

    void set(DtuReg reg, reg_t value, RegAccess access = RegAccess::DTU);

    reg_t get(CmdReg reg, RegAccess access = RegAccess::DTU) const;

    void set(CmdReg reg, reg_t value, RegAccess access = RegAccess::DTU);

    SendEp getSendEp(unsigned epId, bool print = true) const;

    void setSendEp(unsigned epId, const SendEp &ep);

    RecvEp getRecvEp(unsigned epId, bool print = true) const;

    void setRecvEp(unsigned epId, const RecvEp &ep);

    MemEp getMemEp(unsigned epId, bool print = true) const;

    /// returns which command registers have been written
    Result handleRequest(PacketPtr pkt, bool isCpuRequest);

    const std::string name() const { return _name; }

    Addr getSize() const;

  private:

    reg_t get(unsigned epId, size_t idx) const;

    void set(unsigned epId, size_t idx, reg_t value);

    EpType getEpType(unsigned epId) const;

    void printEpAccess(unsigned epId, bool read, bool cpu) const;

  private:

    std::vector<reg_t> dtuRegs;

    std::vector<reg_t> cmdRegs;

    std::vector<std::vector<reg_t>> epRegs;

    const unsigned numEndpoints;

    // used for debug messages (DPRINTF)
    const std::string _name;

  private:

    static const char *dtuRegNames[];
    static const char *cmdRegNames[];
    static const char *epTypeNames[];
};

#endif // __MEM_DTU_REGFILE_HH__
