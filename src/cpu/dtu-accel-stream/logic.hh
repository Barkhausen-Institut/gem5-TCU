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

#ifndef __CPU_DTU_ACCEL_STREAM_LOGIC_HH__
#define __CPU_DTU_ACCEL_STREAM_LOGIC_HH__

#include "params/AccelLogic.hh"
#include "cpu/dtu-accel/accelerator.hh"
#include "cpu/dtu-accel-stream/algorithm.hh"
#include "sim/system.hh"

class DtuAccelStream;

class AccelLogic : public MemObject
{
    class CpuPort : public MasterPort
    {
      private:
        AccelLogic& logic;
      public:
        CpuPort(const std::string& _name, AccelLogic* _logic)
            : MasterPort(_name, _logic), logic(*_logic)
        { }
      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

  public:

    enum State
    {
        LOGIC_PULL,
        LOGIC_PUSH,
        LOGIC_DONE,
    };

    explicit AccelLogic(const AccelLogicParams *p);

    Port& getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void setAccelerator(DtuAccelStream *_accel) {
        accel = _accel;
    }

    std::string stateName() const;

    bool hasStateChanged() const { return stateChanged; }

    size_t outDataSize() const { return outSize; }

    void start(Addr _offset, Addr _dataSize, Cycles _compTime);

   private:
    void tick();
    void handleMemResp(PacketPtr pkt);

    bool sendPkt(PacketPtr pkt);
    void recvRetry();

    EventWrapper<AccelLogic, &AccelLogic::tick> tickEvent;

    CpuPort port;
    PacketPtr retryPkt;

    DtuAccelStream *accel;
    DtuAccelStreamAlgo *algo;
    State state;
    bool stateChanged;
    Cycles compTime;
    Cycles opStart;
    Addr dataSize;
    Addr outSize;
    Addr offset;
    Addr pos;
    Addr pullSize;
    uint8_t *pullData;
};

#endif /* __CPU_DTU_ACCEL_STREAM_LOGIC_HH__ */
