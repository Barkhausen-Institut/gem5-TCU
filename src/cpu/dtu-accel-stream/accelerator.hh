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
#include "cpu/dtu-accel-stream/ctxsw.hh"
#include "cpu/dtu-accel-stream/logic.hh"
#include "cpu/dtu-accel/accelerator.hh"
#include "mem/dtu/connector/base.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuAccelStream : public DtuAccel
{
  public:

    static const Addr BUF_ADDR          = 0x6000;

    static const unsigned EP_RECV       = 7;
    static const unsigned EP_INPUT      = 8;
    static const unsigned EP_OUTPUT     = 9;
    static const unsigned EP_SEND       = 10;
    static const unsigned EP_CTX        = 11;
    static const unsigned CAP_RGATE     = 2;
    static const unsigned CAP_SGATE     = 3;

    static const size_t MSG_SIZE        = 64;

  public:
    DtuAccelStream(const DtuAccelStreamParams *p);

    void wakeup() override;

    void interrupt() override;

    void reset() override;

    size_t stateSize() const override { return bufSize; }
    size_t contextSize() const override { return sizeof(ctx); }
    void *context() override { return &ctx; }

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
        READ_DATA,
        READ_DATA_WAIT,
        COMPUTE,
        WRITE_DATA,
        WRITE_DATA_WAIT,

        MSG_STORE,
        MSG_SEND,
        MSG_WAIT,
        MSG_IDLE,
        MSG_ERROR,

        REPLY_SEND,
        REPLY_WAIT,
        REPLY_ERROR,

        CTXSW,

        SYSCALL,
    };

    enum class Command
    {
        INIT,
        UPDATE,
    };

    std::string getStateName() const;

    bool irqPending;
    bool memPending;

    State state;
    State lastState;

    struct
    {
        Addr msgAddr;
        bool eof;
        Addr off;
        Addr inOff;
        Addr outOff;
        Addr outSize;
        Addr reportSize;
        Cycles compTime;
        Addr accSize;
        Addr dataSize;
        Addr lastSize;
    } ctx;

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
            uint64_t off;
            uint64_t len;
            uint64_t eof;
        } M5_ATTR_PACKED msg;
    } M5_ATTR_PACKED msg;

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
            uint8_t dummy;
        } M5_ATTR_PACKED msg;
    } M5_ATTR_PACKED reply;

    size_t bufSize;
    SyscallSM sysc;
    State syscNext;
    YieldSM yield;
    AccelLogic logic;
    ContextSwitch ctxsw;
};

#endif // __CPU_DTU_ACCEL_STREAM_ACCELERATOR_HH__
