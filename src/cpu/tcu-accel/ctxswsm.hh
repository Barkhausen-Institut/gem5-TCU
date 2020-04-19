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

#ifndef __CPU_TCU_ACCEL_CTXSWSM_HH__
#define __CPU_TCU_ACCEL_CTXSWSM_HH__

#include "cpu/tcu-accel/accelerator.hh"
#include "sim/system.hh"

class AccelCtxSwSM
{
    static const uint64_t OUR_VPE   = 0xFFFF;

    static const size_t RBUF_ADDR   = 0x1FF000;

    static const unsigned EP_RECV   = 2;
    static const size_t MSG_SIZE    = 64;

    enum Operation
    {
        VPE_CTRL,
        MAP,
        REM_MSGS,
    };

    enum VPECtrl
    {
        INIT,
        START,
        STOP,
    };

    enum State
    {
        FETCH_MSG,
        READ_MSG_ADDR,
        READ_MSG,

        STORE_REPLY,
        SEND_REPLY,
        REPLY_WAIT,
    };

    struct
    {
        uint64_t res;
    } M5_ATTR_PACKED reply;

  public:

    explicit AccelCtxSwSM(TcuAccel *_accel);

    std::string stateName() const;

    bool hasStateChanged() const { return stateChanged; }

    void restart() { state = FETCH_MSG; }

    PacketPtr tick();

    bool handleMemResp(PacketPtr pkt);

   private:

    TcuAccel *accel;
    State state;
    bool stateChanged;
    bool switched;
    Addr msgAddr;
    uint64_t vpe_id;
};

#endif /* __CPU_TCU_ACCEL_CTXSWSM_HH__ */
