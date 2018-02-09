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

#ifndef __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__
#define __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__

#include "params/DtuAccelHash.hh"
#include "cpu/dtu-accel/accelerator.hh"
#include "cpu/dtu-accel-hash/algorithm.hh"
#include "mem/dtu/connector/base.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuAccelHash : public DtuAccel
{
  public:
    DtuAccelHash(const DtuAccelHashParams *p);

    void interrupt() override;

    void reset() override;

    size_t stateSize() const override { return bufSize; }
    size_t contextSize() const override { return sizeof(hash); }
    void *context() override { return &hash; }

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
        CTX_FLAGS,
        CTX_RESTORE,
        CTX_RESTORE_WAIT,
        CTX_RESTORE_READ,
        CTX_RESTORE_DONE,

        SYSCALL,
    };

    enum class Command
    {
        INIT,
        UPDATE,
        FINISH
    };

    size_t getStateSize() const;

    std::string getStateName() const;

    size_t bufSize;

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

    SyscallSM sysc;
    State syscNext;
    YieldSM yield;
};

#endif // __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__
