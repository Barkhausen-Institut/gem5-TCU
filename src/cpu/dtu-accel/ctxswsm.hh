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

#ifndef __CPU_DTU_ACCEL_CTXSWSM_HH__
#define __CPU_DTU_ACCEL_CTXSWSM_HH__

#include "cpu/dtu-accel/accelerator.hh"
#include "sim/system.hh"

class AccelCtxSwSM
{
  public:

    enum State
    {
        SAVE,
        SAVE_WRITE,
        SAVE_SEND,
        SAVE_WAIT,
        SAVE_DONE,
        WAIT,

        CHECK,
        FLAGS,
        RESTORE,
        RESTORE_WAIT,
        RESTORE_READ,
        RESTORE_DONE,
    };

    explicit AccelCtxSwSM(DtuAccel *_accel);

    std::string stateName() const;

    bool hasStateChanged() const { return stateChanged; }

    bool isWaiting() const { return state == WAIT; }
    void restart() { state = CHECK; }

    PacketPtr tick();

    bool handleMemResp(PacketPtr pkt);

   private:

    DtuAccel *accel;
    State state;
    bool stateChanged;
    Addr offset;
    bool ctxSwPending;
    bool switched;
};

#endif /* __CPU_DTU_ACCEL_CTXSWSM_HH__ */
