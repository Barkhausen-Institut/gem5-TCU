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

#include "cpu/dtu-accel/ctxswsm.hh"

AccelCtxSwSM::AccelCtxSwSM(DtuAccel *_accel)
    : accel(_accel),
      state(CHECK), stateChanged(), switched()
{
}

std::string
AccelCtxSwSM::stateName() const
{
    const char *names[] =
    {
        "CHECK",
        "FLAGS",
        "DONE",
    };
    return names[static_cast<size_t>(state)];
}

PacketPtr
AccelCtxSwSM::tick()
{
    PacketPtr pkt = nullptr;

    switch(state)
    {
        case State::CHECK:
        {
            Addr regAddr = accel->getRegAddr(ReqReg::EXT_REQ);
            pkt = accel->createDtuRegPkt(
                regAddr, sizeof(uint64_t), MemCmd::WriteReq
            );
            *pkt->getPtr<uint64_t>() = 0;
            break;
        }
        case State::FLAGS:
        {
            pkt = accel->createPacket(
                DtuAccel::PEMUX_FLAGS, sizeof(uint64_t), MemCmd::ReadReq
            );
            break;
        }
        case State::DONE:
        {
            pkt = accel->createPacket(
                DtuAccel::PEMUX_FLAGS, sizeof(uint64_t), MemCmd::WriteReq
            );
            *pkt->getPtr<uint64_t>() = DtuAccel::PEMuxCtrl::SIGNAL;
            break;
        }
    }

    return pkt;
}

bool
AccelCtxSwSM::handleMemResp(PacketPtr pkt)
{
    auto lastState = state;

    switch(state)
    {
        case State::CHECK:
        {
            state = State::FLAGS;
            break;
        }
        case State::FLAGS:
        {
            uint64_t val = *pkt->getConstPtr<uint64_t>();
            if (val & DtuAccel::PEMuxCtrl::RESTORE)
                switched = true;
            if (val & DtuAccel::PEMuxCtrl::WAITING)
                state = State::DONE;
            else
            {
                state = State::CHECK;
                return true;
            }
            break;
        }
        case State::DONE:
        {
            if (switched)
            {
                accel->setSwitched();
                switched = false;
            }
            state = State::CHECK;
            return true;
        }
    }

    stateChanged = state != lastState;

    return false;
}
