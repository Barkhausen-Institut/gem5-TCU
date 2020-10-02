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

#ifndef __CPU_TCU_ACCEL_INDIR_HH__
#define __CPU_TCU_ACCEL_INDIR_HH__

#include "params/TcuAccelInDir.hh"
#include "cpu/tcu-accel/accelerator.hh"
#include "cpu/tcu-accel/ctxswsm.hh"
#include "cpu/tcu-accel/syscallsm.hh"
#include "cpu/tcu-accel/yieldsm.hh"
#include "mem/tcu/connector/base.hh"
#include "mem/tcu/reg_file.hh"
#include "sim/system.hh"

class TcuAccelInDir : public TcuAccel
{
    static const unsigned EP_OUT        = 16;
    static const unsigned EP_RECV       = 17;

    static const size_t MSG_SIZE        = 64;
    static const Addr RBUF_ADDR         = 0x3FFF00;
    static const Addr MSG_ADDR          = 0x2000;
    static const Addr BUF_ADDR          = 0x8000;
    static const size_t BLOCK_SIZE      = 1024;

  public:
    TcuAccelInDir(const TcuAccelInDirParams *p);

    void wakeup() override;

    void interrupt() override;

    void reset() override;

    Addr rbufAddr() const { return RBUF_ADDR + offset; }
    Addr sendMsgAddr() const override { return MSG_ADDR + offset; }
    Addr bufferAddr() const override { return BUF_ADDR + offset; }
    void setSwitched() override {}

  private:

    /// main simulation loop
    void tick() override;

    void completeRequest(PacketPtr pkt) override;

    enum class State
    {
        IDLE,

        FETCH_MSG,
        FETCH_MSG_WAIT,
        READ_MSG_ADDR,
        READ_MSG,

        WRITE_DATA,
        WRITE_DATA_WAIT,

        STORE_REPLY,
        SEND_REPLY,
        REPLY_WAIT,

        CTXSW,
    };

    std::string getStateName() const;

    size_t bufSize;

    bool irqPending;
    bool memPending;

    State state;
    State lastState;

    Addr msgAddr;
    Addr dataSize;

    enum Operation
    {
        COMPUTE,
        FORWARD,
    };

    struct
    {
        uint64_t count;
    } M5_ATTR_PACKED reply;

    YieldSM yield;
    AccelCtxSwSM ctxsw;
};

#endif // __CPU_TCU_ACCEL_INDIR_HH__
