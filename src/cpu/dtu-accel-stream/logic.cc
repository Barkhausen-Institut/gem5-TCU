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

#include "debug/DtuAccelStream.hh"
#include "cpu/dtu-accel-stream/accelerator.hh"
#include "cpu/dtu-accel-stream/algorithm_fft.hh"
#include "cpu/dtu-accel-stream/algorithm_toupper.hh"
#include "cpu/dtu-accel-stream/logic.hh"

AccelLogic::AccelLogic(DtuAccel *_accel, int _algo)
    : accel(_accel), algo(), state(),
      compTime(), opStart(), dataSize(), offset(), pullSize(), pullData()
{
    if (_algo == 0)
        algo = new DtuAccelStreamAlgoFFT();
    else if(_algo == 1)
        algo = new DtuAccelStreamAlgoToUpper();
    else
        panic("Unknown algorithm %d\n", _algo);
}

std::string
AccelLogic::stateName() const
{
    const char *names[] = {"PULL", "PUSH"};
    return names[static_cast<size_t>(state)];
}

PacketPtr
AccelLogic::tick()
{
    PacketPtr pkt = nullptr;

    switch(state)
    {
        case State::LOGIC_PULL:
        {
            if (offset == 0)
                opStart = accel->curCycle();
            size_t rem = dataSize - offset;
            pullSize = std::min(accel->chunkSize, rem);
            pkt = accel->createPacket(
                DtuAccelStream::BUF_ADDR + offset,
                pullSize,
                MemCmd::ReadReq
            );
            break;
        }
        case State::LOGIC_PUSH:
        {
            pkt = accel->createPacket(
                DtuAccelStream::BUF_ADDR + offset,
                pullData,
                pullSize,
                MemCmd::WriteReq
            );
            break;
        }
    }

    return pkt;
}

bool
AccelLogic::handleMemResp(PacketPtr pkt, Cycles *delay)
{
    *delay = Cycles(0);

    switch(state)
    {
        case State::LOGIC_PULL:
        {
            // load data to be able to change it and write it back
            pullData = new uint8_t[pullSize];

            // execute the algorithm
            algo->execute(pullData,
                          pkt->getConstPtr<uint8_t>(),
                          pullSize);

            state = State::LOGIC_PUSH;
            break;
        }
        case State::LOGIC_PUSH:
        {
            offset += pullSize;

            if (offset == dataSize)
            {
                // decrease it by the time we've already spent reading the
                // data from SPM, because that's already included in the
                // BLOCK_TIME.
                // TODO if we use caches, this is not correct
                *delay = algo->getDelay(compTime, dataSize);
                DPRINTF(DtuAccelStream, "%s for %luB took %llu cycles\n",
                    algo->name(), dataSize, *delay);

                if (*delay > (accel->curCycle() - opStart))
                    *delay = *delay - (accel->curCycle() - opStart);
                else
                    *delay = Cycles(1);

                return true;
            }
            else
                state = State::LOGIC_PULL;
            break;
        }
    }

    return false;
}
