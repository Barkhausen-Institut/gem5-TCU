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

#ifndef __CPU_DTU_ACCEL_ALADDIN_HH_
#define __CPU_DTU_ACCEL_ALADDIN_HH_

#include "params/DtuAccelAladdin.hh"
#include "cpu/dtu-accel/accelerator.hh"
#include "cpu/dtu-accel/ctxswsm.hh"
#include "cpu/dtu-accel/syscallsm.hh"
#include "cpu/dtu-accel/yieldsm.hh"
#include "mem/dtu/connector/base.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuAccelAladdin : public DtuAccel
{
    static const unsigned EP_RECV       = 7;
    static const unsigned EP_CTX        = 8;
    static const unsigned EP_DATA       = 9;
    static const unsigned CAP_RBUF      = 2;

    static const size_t MSG_SIZE        = 256;
    static const Addr BUF_ADDR          = 0x8000;

  public:
    DtuAccelAladdin(const DtuAccelAladdinParams *p);

    void interrupt() override;

    void reset() override;

    void signalFinished(size_t off) override;

    Addr sendMsgAddr() const override { return BUF_ADDR; }
    Addr bufferAddr() const override { return BUF_ADDR; }
    int contextEp() const override { return EP_CTX; }
    size_t stateSize() const override { return 0; }
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
        COMPUTE,
        STORE_REPLY,
        SEND_REPLY,
        REPLY_WAIT,
        REPLY_ERROR,

        CTXSW,

        SYSCALL,
    };

    struct InvokeMessage
    {
        struct
        {
            uint64_t addr;
            uint64_t size;
        } M5_ATTR_PACKED arrays[8];
        uint64_t array_count;
        uint64_t iterations;
    } M5_ATTR_PACKED;

    std::string getStateName() const;

    bool irqPending;
    bool ctxSwPending;
    bool memPending;

    State state;
    State lastState;

    struct Context {
        uint64_t msgAddr;
        uint64_t msgOff;
        InvokeMessage msg;
        uint64_t iteration;
        uint64_t interrupted;
        uint64_t trace_off;
        char _pad[8];
    } M5_ATTR_PACKED ctx;

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
        } M5_ATTR_PACKED msg;
    } M5_ATTR_PACKED reply;

    unsigned accelId;

    SyscallSM sysc;
    State syscNext;
    YieldSM yield;
    AccelCtxSwSM ctxsw;
    bool ctxSwPerformed;
};

#endif // __CPU_DTU_ACCEL_ALADDIN_HH_
