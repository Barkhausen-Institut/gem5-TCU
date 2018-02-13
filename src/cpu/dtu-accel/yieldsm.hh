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

#ifndef __CPU_DTU_ACCEL_YIELDSM_HH__
#define __CPU_DTU_ACCEL_YIELDSM_HH__

#include "cpu/dtu-accel/accelerator.hh"
#include "cpu/dtu-accel/syscallsm.hh"
#include "sim/system.hh"

class YieldSM
{
    struct Syscall
    {
        uint64_t opcode;
        uint64_t vpe_sel;
        uint64_t op;
        uint64_t arg;
    } M5_ATTR_PACKED;

  public:

    enum State
    {
        YLD_CHECK,
        YLD_WAIT,
        YLD_REPORT,
        YLD_SYSCALL,
        YLD_SLEEP,
    };

    explicit YieldSM(DtuAccel *_accel, SyscallSM *_syscsm)
        : accel(_accel), syscsm(_syscsm), state() {}

    std::string stateName() const;

    bool hasStateChanged() const { return stateChanged; }

    void start(bool check = true)
    {
        state = check ? YLD_CHECK : YLD_SLEEP;
    }

    PacketPtr tick();

    bool handleMemResp(PacketPtr pkt);

    DtuAccel *accel;
    SyscallSM *syscsm;
    Syscall syscall;
    uint64_t report;
    Cycles yieldStart;
    State state;
    bool stateChanged;
};

#endif /* __CPU_DTU_ACCEL_YIELDSM_HH__ */
