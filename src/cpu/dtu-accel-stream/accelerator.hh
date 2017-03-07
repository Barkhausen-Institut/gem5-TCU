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
#include "cpu/dtu-accel-stream/algorithm.hh"
#include "cpu/dtu-accel/accelerator.hh"
#include "mem/dtu/connector/base.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuAccelStream : public DtuAccel
{
  public:
    DtuAccelStream(const DtuAccelStreamParams *p);

    void interrupt() override;

    void wakeup() override;

    void reset() override;

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
        PULL_DATA,
        PUSH_DATA,
        WRITE_DATA,
        WRITE_DATA_WAIT,
        STORE_REPLY,
        SEND_REPLY,
        REPLY_WAIT,
        REPLY_ERROR,

        CTX_SAVE,
        CTX_SAVE_DONE,
        CTX_WAIT,

        CTX_CHECK,
        CTX_RESTORE,

        SYSCALL,
    };

    DtuAccelStreamAlgo *algo;

    std::string getStateName() const;

    size_t bufSize;

    bool irqPending;
    bool ctxSwPending;
    bool memPending;

    State state;

    Addr msgAddr;

    bool autonomous;
    Addr inOff;
    Addr outOff;
    Addr dataOff;
    Addr dataSize;
    Addr lastSize;
    Addr streamOff;
    Addr lastPullSize;
    uint8_t *lastData;
    Cycles opStart;

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

    SyscallSM sysc;
    State syscNext;
    YieldSM yield;
};

#endif // __CPU_DTU_ACCEL_STREAM_ACCELERATOR_HH__
