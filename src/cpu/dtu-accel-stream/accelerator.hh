/*
 * Copyright (c) 2016, Nils Asmussen
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

#ifndef __CPU_DTU_ACCEL_STREAM_ACCELERATOR_HH__
#define __CPU_DTU_ACCEL_STREAM_ACCELERATOR_HH__

#include "params/DtuAccelStream.hh"
#include "cpu/dtu-accel-stream/logic.hh"
#include "cpu/dtu-accel/accelerator.hh"
#include "cpu/dtu-accel/ctxswsm.hh"
#include "cpu/dtu-accel/syscallsm.hh"
#include "cpu/dtu-accel/yieldsm.hh"
#include "mem/dtu/connector/base.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuAccelStream : public DtuAccel
{
  public:

    static const Addr MSG_ADDR          = 0x2000;
    static const Addr BUF_ADDR          = 0x8000;

    static const unsigned EP_RECV       = 5;
    static const unsigned EP_IN_SEND    = 6;
    static const unsigned EP_IN_MEM     = 7;
    static const unsigned EP_OUT_SEND   = 8;
    static const unsigned EP_OUT_MEM    = 9;
    static const unsigned EP_CTX        = 10;

    static const unsigned CAP_IN       = 64;
    static const unsigned CAP_OUT      = 65;
    static const unsigned CAP_RECV     = 66;

    static const uint64_t LBL_IN_REQ    = 1;
    static const uint64_t LBL_IN_REPLY  = 2;
    static const uint64_t LBL_OUT_REQ   = 3;
    static const uint64_t LBL_OUT_REPLY = 4;

    static const size_t MSG_SIZE        = 64;

    static const uint64_t NO_SUBMIT     = 0xFFFFFFFFFFFFFFFF;

  public:
    DtuAccelStream(const DtuAccelStreamParams *p);

    void wakeup() override;

    void interrupt() override;

    void reset() override;

    void logicFinished();

    Addr sendMsgAddr() const override { return MSG_ADDR; }
    Addr bufferAddr() const override { return BUF_ADDR; }
    int contextEp() const override { return EP_CTX; }
    size_t stateSize() const override { return bufSize; }
    size_t contextSize() const override { return sizeof(ctx); }
    void *context() override { return &ctx; }
    void setSwitched() override { ctxSwPerformed = true; }

  private:

    /// main simulation loop
    void tick() override;

    void completeRequest(PacketPtr pkt) override;

    enum class State
    {
        IDLE,

        FETCH_MSG,
        READ_MSG_ADDR,
        READ_MSG,

        INOUT_START,
        INOUT_SEND,
        INOUT_SEND_WAIT,
        INOUT_SEND_ERROR,
        INOUT_ACK,

        READ_DATA,
        READ_DATA_WAIT,

        WRITE_DATA,
        WRITE_DATA_WAIT,

        REPLY_STORE,
        REPLY_SEND,
        REPLY_WAIT,
        REPLY_ERROR,

        CTXSW,

        SYSCALL,

        SUBMIT_START,
        SUBMIT_SEND,
        SUBMIT_SEND_WAIT,
        SUBMIT_SEND_ERROR,

        EXIT_ACK,
        EXIT,
    };

    enum class Command
    {
        STAT,
        SEEK,
        READ,
        WRITE,
    };

    std::string getStateName() const;

    bool irqPending;
    bool memPending;

    State state;
    State lastState;
    uint16_t lastFlags;
    Addr bufOff;

    enum Flags
    {
        OUTPUT      = 0x1,
        WAIT        = 0x2,
        SEEN_EOF    = 0x4,
        SEEN_SUBMIT = 0x8,
        EXIT        = 0x10,
        COMP        = 0x20,
        COMPDONE    = 0x40,
        FETCHED     = 0x80,
        TRANSFER    = 0x100,
        INSYSC      = 0x200,
    };

    struct
    {
        uint32_t flags;
        uint32_t inMask : 2,
                 outMask : 2,
                 : 28;
        uint32_t outMaskLen[2];
        uint64_t compTime;
        uint64_t msgAddr;
        uint64_t inReqAddr;
        uint64_t outReqAddr;
        uint64_t commitOff;
        uint64_t commitLen;
        uint64_t inOff;
        uint64_t inPos;
        uint64_t inLen;
        uint64_t outOff;
        uint64_t outPos;
        uint64_t outLen;
        uint64_t lastSize;
        uint64_t nextSysc;
    } M5_ATTR_PACKED ctx;

    struct
    {
        struct
        {
            uint64_t opcode;
            uint64_t sgate_sel;
            uint64_t rgate_sel;
            uint64_t len;
            uint64_t rlabel;
            uint64_t event;
        } M5_ATTR_PACKED sys;
        struct
        {
            uint64_t cmd;
            uint64_t submit;
        } M5_ATTR_PACKED msg;
    } M5_ATTR_PACKED rdwr_msg;

    struct
    {
        uint64_t opcode;
        uint64_t vpe_sel;
        uint64_t op;
        uint64_t arg;
    } M5_ATTR_PACKED exit_msg;

    struct
    {
        struct
        {
            uint64_t opcode;
            uint64_t rgate_sel;
            uint64_t msgaddr;
            uint64_t len;
            uint64_t event;
        } M5_ATTR_PACKED sys;
        struct
        {
            uint64_t err;
            uint64_t off;
            uint64_t len;
        } M5_ATTR_PACKED msg;
    } M5_ATTR_PACKED reply;

    size_t bufSize;
    SyscallSM sysc;
    State syscNext;
    Addr replyAddr;
    State replyNext;
    YieldSM yield;
    AccelLogic *logic;
    AccelCtxSwSM ctxsw;
    bool ctxSwPerformed;
};

#endif // __CPU_DTU_ACCEL_STREAM_ACCELERATOR_HH__
