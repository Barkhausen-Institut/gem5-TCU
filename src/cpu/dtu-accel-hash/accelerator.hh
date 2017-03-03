/*
 * Copyright (c) 2016, Nils Asmussen
 * Copyright (c) 2015, Christian Menard
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

#ifndef __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__
#define __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__

#include "params/DtuAccelHash.hh"
#include "cpu/dtu-accel-hash/algorithm.hh"
#include "mem/dtu/connector/base.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuAccelHash : public MemObject
{
  public:
    DtuAccelHash(const DtuAccelHashParams *p);

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    void setConnector(BaseConnector *con)
    {
        connector = con;
    }

    void interrupt();

    void wakeup();

    void reset();

  protected:

    /// main simulation loop
    void tick();

    class CpuPort : public MasterPort
    {
      private:
        DtuAccelHash& dtutest;
      public:
        CpuPort(const std::string& _name, DtuAccelHash* _dtutest)
            : MasterPort(_name, _dtutest), dtutest(*_dtutest)
        { }
      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    enum class State
    {
        IDLE,
        IDLE_WAIT,
        IDLE_REPORT,
        IDLE_START,

        FETCH_MSG,
        READ_MSG_ADDR,
        READ_MSG,
        READ_DATA,
        READ_DATA_WAIT,
        HASH_DATA,
        STORE_REPLY,
        SEND_REPLY,
        REPLY_WAIT,
        REPLY_ERROR,

        CTX_SAVE,
        CTX_SAVE_WRITE,
        CTX_SAVE_SEND,
        CTX_SAVE_WAIT,
        CTX_SAVE_DONE,
        CTX_WAIT,

        CTX_CHECK,
        CTX_RESTORE,
        CTX_RESTORE_WAIT,
        CTX_RESTORE_READ,
        CTX_RESTORE_DONE,

        SYSCALL,
    };

    class SyscallSM
    {
      public:

        enum State
        {
            SYSC_SEND,
            SYSC_WAIT,
            SYSC_FETCH,
            SYSC_READ_ADDR,
            SYSC_ACK,
        };

        explicit SyscallSM(DtuAccelHash *_accel)
            : accel(_accel), state(), replyAddr(), syscallSize() {}

        const char *stateName() const;

        void start(Addr size)
        {
            syscallSize = size;
            state = SYSC_SEND;
        }

        PacketPtr tick();

        bool handleMemResp(PacketPtr pkt);

       private:

        DtuAccelHash *accel;
        State state;
        Addr replyAddr;
        Addr syscallSize;
    };

    enum RCTMuxCtrl
    {
        NONE        = 0,
        STORE       = 1 << 0, // store operation required
        RESTORE     = 1 << 1, // restore operation required
        WAITING     = 1 << 2, // set by the kernel if a signal is required
        SIGNAL      = 1 << 3, // used to signal completion to the kernel
    };

    enum class Command
    {
        INIT,
        UPDATE,
        FINISH
    };

    PacketPtr createPacket(Addr paddr, size_t size, MemCmd cmd);

    PacketPtr createDtuRegPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    PacketPtr createDtuCmdPkt(uint64_t cmd, uint64_t data, uint64_t size, uint64_t off);

    void freePacket(PacketPtr pkt);

    bool sendPkt(PacketPtr pkt);

    void completeRequest(PacketPtr pkt);

    void recvRetry();

    static Addr getRegAddr(DtuReg reg);

    static Addr getRegAddr(CmdReg reg);

    static Addr getRegAddr(unsigned reg, unsigned epid);

    size_t getStateSize() const;

    System *system;

    EventWrapper<DtuAccelHash, &DtuAccelHash::tick> tickEvent;

    CpuPort port;

    size_t bufSize;
    size_t maxDataSize;

    bool haveVM;
    Addr chunkSize;

    bool irqPending;
    bool ctxSwPending;
    bool memPending;

    State state;
    DtuAccelHashAlgorithm hash;

    Addr ctxOffset;

    Addr msgAddr;
    Addr hashOff;
    Addr lastSize;

    Cycles hashStart;
    Cycles yieldStart;

    size_t replyOffset;
    size_t replySize;
    struct
    {
        struct
        {
            uint64_t opcode;
            uint64_t cap;
            uint64_t msgaddr;
            uint64_t len;
            uint64_t event;
        } M5_ATTR_PACKED sys;
        struct
        {
            uint64_t res;
            uint8_t bytes[64];
        } M5_ATTR_PACKED msg;
    } M5_ATTR_PACKED reply;

    struct
    {
        uint64_t opcode;
        uint64_t vpe_sel;
        uint64_t op;
        uint64_t arg;
    } M5_ATTR_PACKED yieldSyscall;
    uint64_t yieldReport;

    SyscallSM sysc;
    State syscNext;

    /// Request id for all generated traffic
    MasterID masterId;

    unsigned int id;

    const bool atomic;

    Addr reg_base;

    /// Stores the Packet for later retry
    PacketPtr retryPkt;

    BaseConnector *connector;
};

#endif // __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__
