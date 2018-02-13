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

#include "cpu/dtu-accel/yieldsm.hh"

std::string
YieldSM::stateName() const
{
    std::ostringstream os;
    static const char *names[] =
    {
        "CHECK", "WAIT", "REPORT", "SYSCALL", "SLEEP"
    };
    os << names[static_cast<size_t>(state)];
    if (state == YLD_SYSCALL)
        os << ":" << syscsm->stateName();
    return os.str();
}

PacketPtr
YieldSM::tick()
{
    PacketPtr pkt = nullptr;

    switch(state)
    {
        case State::YLD_CHECK:
        {
            pkt = accel->createPacket(
                DtuAccel::RCTMUX_YIELD, sizeof(uint64_t), MemCmd::ReadReq
            );
            break;
        }
        case State::YLD_WAIT:
        {
            yieldStart = accel->curCycle();
            pkt = accel->createDtuCmdPkt(Dtu::Command::SLEEP, 0, 0, 0, report);
            break;
        }
        case State::YLD_REPORT:
        {
            syscall.opcode = 10;   /* VPE_CTRL */
            syscall.vpe_sel = 0;   /* self */
            syscall.op = 2;        /* VCTRL_YIELD */
            syscall.arg = 0;       /* unused */

            pkt = accel->createPacket(
                DtuAccel::MSG_ADDR, sizeof(syscall), MemCmd::WriteReq
            );
            memcpy(pkt->getPtr<void>(), &syscall, sizeof(syscall));
            break;
        }
        case State::YLD_SYSCALL:
        {
            pkt = syscsm->tick();
            break;
        }
        case State::YLD_SLEEP:
        {
            pkt = accel->createDtuCmdPkt(Dtu::Command::SLEEP, 0, 0, 0, 0);
            break;
        }
    }

    return pkt;
}

bool
YieldSM::handleMemResp(PacketPtr pkt)
{
    auto lastState = state;

    switch(state)
    {
        case State::YLD_CHECK:
        {
            report = *pkt->getConstPtr<uint64_t>();
            if (report > 0)
                state = State::YLD_WAIT;
            else
                state = State::YLD_SLEEP;
            break;
        }
        case State::YLD_WAIT:
        {
            if (accel->curCycle() < yieldStart + report)
                return true;

            state = State::YLD_REPORT;
            break;
        }
        case State::YLD_REPORT:
        {
            syscsm->start(sizeof(syscall));
            state = State::YLD_SYSCALL;
            break;
        }
        case State::YLD_SYSCALL:
        {
            if(syscsm->handleMemResp(pkt))
                state = State::YLD_SLEEP;
            break;
        }
        case State::YLD_SLEEP:
        {
            return true;
        }
    }

    stateChanged = state != lastState;

    return false;
}
