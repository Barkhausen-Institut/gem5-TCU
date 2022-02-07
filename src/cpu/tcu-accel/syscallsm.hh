/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2021 Nils Asmussen, Barkhausen Institut
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

#ifndef __CPU_TCU_ACCEL_SYSCALLSM_HH__
#define __CPU_TCU_ACCEL_SYSCALLSM_HH__

#include "cpu/tcu-accel/accelerator.hh"
#include "sim/system.hh"

class SyscallSM
{
    static const size_t RBUF_ADDR = 0x3FB000;

  public:
    enum Operation
    {
        // capability creations
        CREATE_SRV,
        CREATE_SESS,
        CREATE_MGATE,
        CREATE_RGATE,
        CREATE_SGATE,
        CREATE_MAP,
        CREATE_VPE,
        CREATE_SEM,
        ALLOC_EPS,

        // capability operations
        ACTIVATE,
        SET_PMP,
        VPE_CTRL,
        VPE_WAIT,
        DERIVE_MEM,
        DERIVE_KMEM,
        DERIVE_PE,
        DERIVE_SRV,
        GET_SESS,
        MGATE_REGION,
        KMEM_QUOTA,
        PE_QUOTA,
        PE_SET_QUOTA,
        SEM_CTRL,

        // capability exchange
        DELEGATE,
        OBTAIN,
        EXCHANGE,
        REVOKE,

        // misc
        RESET_STATS,
        NOOP,
    };

    enum VPEOp
    {
        VCTRL_INIT,
        VCTRL_START,
        VCTRL_STOP,
    };

    enum State
    {
        SYSC_SEND,
        SYSC_SEND_WAIT,
        SYSC_FETCH,
        SYSC_FETCH_WAIT,
        SYSC_READ_ADDR,
        SYSC_ACK,
        SYSC_ACK_WAIT,
    };

    static const uint64_t VPE_SEL = 2;

    explicit SyscallSM(TcuAccel *_accel)
        : accel(_accel), state(), stateChanged(), waitForReply(), fetched(),
          replyAddr(), syscallSize() {}

    std::string stateName() const;

    bool isWaiting() const { return state == SYSC_FETCH; }

    bool hasStateChanged() const { return stateChanged; }

    void retryFetch() { fetched = false; }

    void start(Addr size, bool wait = true, bool resume = false)
    {
        syscallSize = size;
        state = resume ? SYSC_FETCH : SYSC_SEND;
        fetched = false;
        waitForReply = wait;
    }

    PacketPtr tick();

    bool handleMemResp(PacketPtr pkt);

   private:

    TcuAccel *accel;
    State state;
    bool stateChanged;
    bool waitForReply;
    bool fetched;
    Addr replyAddr;
    Addr syscallSize;
};

#endif /* __CPU_TCU_ACCEL_SYSCALLSM_HH__ */
