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
#include "debug/DtuAccelStreamState.hh"
#include "cpu/dtu-accel-stream/accelerator.hh"
#include "cpu/dtu-accel-stream/algorithm_fft.hh"
#include "cpu/dtu-accel-stream/algorithm_rot13.hh"
#include "cpu/dtu-accel-stream/logic.hh"

AccelLogic::AccelLogic(const AccelLogicParams *p)
    : MemObject(p), tickEvent(this), port("port", this),
      accel(), algo(), state(), stateChanged(),
      compTime(), opStart(), dataSize(), offset(), pos(), pullSize(), pullData()
{
    if (p->algorithm == 0)
        algo = new DtuAccelStreamAlgoFFT();
    else if(p->algorithm == 1)
        algo = new DtuAccelStreamAlgoROT13();
    else
        panic("Unknown algorithm %d\n", p->algorithm);
}

std::string
AccelLogic::stateName() const
{
    const char *names[] = {"PULL", "PUSH", "DONE"};
    return names[static_cast<size_t>(state)];
}

void
AccelLogic::start(Addr _offset, Addr _dataSize, Cycles _compTime)
{
    dataSize = _dataSize;
    compTime = _compTime;
    state = dataSize == 0 ? LOGIC_DONE : LOGIC_PULL;
    offset = _offset;
    pos = 0;
    outSize = 0;
    opStart = curCycle();
    schedule(tickEvent, clockEdge(Cycles(1)));
}

void
AccelLogic::tick()
{
    PacketPtr pkt = nullptr;

    DPRINTF(DtuAccelStreamState, "[%s] tick\n",
        stateName().c_str());

    switch(state)
    {
        case State::LOGIC_PULL:
        {
            size_t rem = dataSize - pos;
            pullSize = std::min(accel->chunkSize, rem);
            pkt = accel->createPacket(
                DtuAccelStream::BUF_ADDR + offset + pos,
                pullSize,
                MemCmd::ReadReq
            );
            break;
        }
        case State::LOGIC_PUSH:
        {
            pkt = accel->createPacket(
                DtuAccelStream::BUF_ADDR + offset + pos,
                pullData,
                pullSize,
                MemCmd::WriteReq
            );
            break;
        }
        case State::LOGIC_DONE:
        {
            accel->logicFinished();
            break;
        }
    }

    if (pkt)
        sendPkt(pkt);
}

void
AccelLogic::handleMemResp(PacketPtr pkt)
{
    Cycles delay(1);

    DPRINTF(DtuAccelStreamState, "[%s] Got response from memory\n",
        stateName().c_str());

    auto lastState = state;

    switch(state)
    {
        case State::LOGIC_PULL:
        {
            // load data to be able to change it and write it back
            pullData = new uint8_t[pullSize];

            // execute the algorithm
            outSize += algo->execute(
                pullData, pkt->getConstPtr<uint8_t>(), pullSize
            );

            state = State::LOGIC_PUSH;
            break;
        }
        case State::LOGIC_PUSH:
        {
            pos += pullSize;

            if (pos == dataSize)
            {
                // decrease it by the time we've already spent reading the
                // data from SPM, because that's already included in the
                // BLOCK_TIME.
                // TODO if we use caches, this is not correct
                delay = algo->getDelay(compTime, dataSize);
                DPRINTF(DtuAccelStream, "%s for %luB took %llu cycles\n",
                    algo->name(), dataSize, delay);

                if (delay > (curCycle() - opStart))
                    delay = delay - (curCycle() - opStart);
                else
                    delay = Cycles(1);
                state = State::LOGIC_DONE;
            }
            else
                state = State::LOGIC_PULL;
            break;
        }
        case State::LOGIC_DONE:
        {
            assert(false);
            break;
        }
    }

    stateChanged = state != lastState;

    schedule(tickEvent, clockEdge(delay));
}

bool
AccelLogic::CpuPort::recvTimingResp(PacketPtr pkt)
{
    logic.handleMemResp(pkt);
    return true;
}

void
AccelLogic::CpuPort::recvReqRetry()
{
    logic.recvRetry();
}

Port &
AccelLogic::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return SimObject::getPort(if_name, idx);
}

bool
AccelLogic::sendPkt(PacketPtr pkt)
{
    if (!port.sendTimingReq(pkt))
    {
        retryPkt = pkt;
        return false;
    }

    return true;
}

void
AccelLogic::recvRetry()
{
    assert(retryPkt);
    if (port.sendTimingReq(retryPkt))
        retryPkt = nullptr;
}

AccelLogic*
AccelLogicParams::create()
{
    return new AccelLogic(this);
}
