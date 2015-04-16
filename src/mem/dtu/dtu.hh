/*
 * Copyright (c) 2015, Christian Menard
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 */

#ifndef __MEM_DTU_DTU_HH__
#define __MEM_DTU_DTU_HH__

#include "mem/dtu/base.hh"
#include "mem/mem_object.hh"
#include "mem/tport.hh"
#include "params/Dtu.hh"

#include <queue>

class Dtu : public BaseDtu
{
  private:

    struct DeferredPacket
    {
        Tick tick;
        PacketPtr pkt;
        DeferredPacket(Tick _tick, PacketPtr _pkt) : tick(_tick), pkt(_pkt) {}
    };

    class DtuMasterPort : public MasterPort
    {
      public:

        DtuMasterPort(const std::string& _name, Dtu& _dtu)
            : MasterPort(_name, &_dtu), dtu(_dtu)
        { }

      protected:

        /**
         * Snooping a coherence request, do nothing.
         */
        virtual void recvTimingSnoopReq(PacketPtr pkt) {}

        Dtu& dtu;

        struct TickEvent : public Event
        {
            std::queue<DeferredPacket> pktQueue;
            Dtu& dtu;

            TickEvent(Dtu& _dtu) : pktQueue(), dtu(_dtu) {}
            const char *description() const { return "DTU tick"; }
            void schedule(PacketPtr _pkt, Tick t);
        };
    };

    class NocMasterPort : public DtuMasterPort
    {
      public:

        NocMasterPort(Dtu& _dtu)
            : DtuMasterPort(_dtu.name() + ".noc_master_port", _dtu),
              tickEvent(_dtu)
        { }

      protected:

        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;

        struct NocTickEvent : public TickEvent
        {
            NocTickEvent(Dtu& _dtu)
                : TickEvent(_dtu) {}
            void process();
            const char *description() const { return "DTU NoC tick"; }
        };

        NocTickEvent tickEvent;
    };

    class ScratchpadPort : public DtuMasterPort
    {
      public:

        ScratchpadPort(Dtu& _dtu)
          : DtuMasterPort(_dtu.name() + ".scratchpad_port", _dtu),
            tickEvent(_dtu)
        { }

      protected:

        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;

        struct SpmTickEvent : public TickEvent
        {
            SpmTickEvent(Dtu& _dtu)
                : TickEvent(_dtu) {}
            void process();
            const char *description() const { return "DTU Scratchpad tick"; }
        };

        SpmTickEvent tickEvent;
    };

    class DtuCpuPort : public SimpleTimingPort
    {
      private:
        Dtu& dtu;

      public:

        DtuCpuPort(const std::string& _name, Dtu& _dtu)
          : SimpleTimingPort(_name, &_dtu), dtu(_dtu)
        { }

      protected:

        Tick recvAtomic(PacketPtr pkt) override;

        AddrRangeList getAddrRanges() const override;
    };

    class DtuSlavePort : public QueuedSlavePort
    {
      private:
        Dtu& dtu;

        RespPacketQueue queueImpl;

      public:

        DtuSlavePort(const std::string& _name, Dtu& _dtu)
          : QueuedSlavePort(_name, &_dtu, queueImpl), dtu(_dtu), queueImpl(_dtu, *this)
        { }

      protected:

        Tick recvAtomic(PacketPtr pkt) override;

        void recvFunctional(PacketPtr pkt) override;

        bool recvTimingReq(PacketPtr pkt) override;

        AddrRangeList getAddrRanges() const override;

    };

    DtuCpuPort     cpu;
    ScratchpadPort scratchpad;
    NocMasterPort  master;
    DtuSlavePort   slave;

    PacketPtr retrySpmPkt;

    PacketPtr retryNocPkt;

    bool nocWaitsForRetry;

    void tick() override;

    EventWrapper<Dtu, &Dtu::tick> tickEvent;

    void recvSpmRetry();

    void recvNocRetry();

    void sendSpmRequest(PacketPtr pkt) override;

    void sendNocRequest(PacketPtr pkt) override;

    bool isSpmPortReady() override;

    bool isNocPortReady() override;

    void sendNocResponse(PacketPtr pkt) override;

  public:

    Dtu(const DtuParams *p);

    void init() override;

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    BaseSlavePort& getSlavePort(const std::string &if_name,
                                PortID idx = InvalidPortID) override;
};

#endif // __MEM_DTU_DTU_HH__
