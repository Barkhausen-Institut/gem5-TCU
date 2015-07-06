/*
 * Copyright (c) 2015, Christian Menard
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

#ifndef __MEM_DTU_BASE_HH__
#define __MEM_DTU_BASE_HH__

#include "mem/mem_object.hh"
#include "mem/qport.hh"
#include "params/BaseDtu.hh"

class BaseDtu : public MemObject
{
  private:

    class DtuMasterPort : public QueuedMasterPort
    {
      protected:

        BaseDtu& dtu;

        ReqPacketQueue reqQueue;

        SnoopRespPacketQueue snoopRespQueue;

      public:

        DtuMasterPort( const std::string& _name, BaseDtu& _dtu);

        virtual void completeRequest(PacketPtr pkt) = 0;

        bool recvTimingResp(PacketPtr pkt) override;
    };

    class NocMasterPort : public DtuMasterPort
    {
      public:

        NocMasterPort(BaseDtu& _dtu)
            : DtuMasterPort(_dtu.name() + ".noc_master_port", _dtu)
        { }

        void completeRequest(PacketPtr pkt) override;
    };

    class ScratchpadPort : public DtuMasterPort
    {
      public:

        ScratchpadPort(BaseDtu& _dtu)
          : DtuMasterPort(_dtu.name() + ".scratchpad_port", _dtu)
        { }

        void completeRequest(PacketPtr pkt) override;
    };

    class DtuSlavePort : public SlavePort
    {
      protected:

        BaseDtu& dtu;

        bool busy;

        bool sendReqRetry;

        PacketPtr respPkt;

        bool waitForRespRetry;

        struct ResponseEvent : public Event
        {
            DtuSlavePort& port;

            ResponseEvent(DtuSlavePort& _port) : port(_port) {}

            void process() override;

            const char* description() const override
            {
                return "DTU ResponseEvent";
            }

            const std::string name() const override { return port.name(); }
        };

        ResponseEvent responseEvent;

        void handleSuccessfulResponse();

      public:

        DtuSlavePort(const std::string _name, BaseDtu& _dtu);

        virtual void handleRequest(PacketPtr pkt) = 0;

        void schedTimingResp(PacketPtr pkt, Tick when);

        Tick recvAtomic(PacketPtr pkt) override;

        void recvFunctional(PacketPtr pkt) override;

        bool recvTimingReq(PacketPtr pkt) override;

        void recvRespRetry() override;
    };

    class CpuPort : public DtuSlavePort
    {
      public:

        CpuPort(BaseDtu& _dtu)
          : DtuSlavePort(_dtu.name() + ".cpu_port", _dtu)
        { }

      protected:

        AddrRangeList getAddrRanges() const override;

        void handleRequest(PacketPtr pkt) override;
    };

    class NocSlavePort : public DtuSlavePort
    {
      public:

        NocSlavePort(BaseDtu& _dtu)
          : DtuSlavePort(_dtu.name() + ".noc_slave_port", _dtu)
        { }

      protected:

        AddrRangeList getAddrRanges() const override;

        void handleRequest(PacketPtr pkt) override;
    };

  protected:

    CpuPort        cpuPort;

    ScratchpadPort scratchpadPort;

    NocMasterPort  nocMasterPort;

    NocSlavePort   nocSlavePort;

    unsigned coreId;

    Addr regFileBaseAddr;

    unsigned nocAddrWidth;

    unsigned nocCoreAddrBits;

    unsigned nocEpAddrBits;

  public:

    BaseDtu(BaseDtuParams* p);

    void init() override;

    BaseSlavePort& getSlavePort(const std::string &if_name, PortID idx) override;

    BaseMasterPort& getMasterPort(const std::string &if_name, PortID idx) override;

    Addr getNocAddr(unsigned coreId, unsigned epId = 0) const;

    void schedNocResponse(PacketPtr pkt, Tick when);

    void schedCpuResponse(PacketPtr pkt, Tick when);

    void schedNocRequest(PacketPtr pkt, Tick when);

    void schedSpmRequest(PacketPtr pkt, Tick when);

    void sendAtomicNocRequest(PacketPtr pkt);

    void sendAtomicSpmRequest(PacketPtr pkt);

    virtual void completeNocRequest(PacketPtr pkt) = 0;

    virtual void completeSpmRequest(PacketPtr pkt) = 0;

    virtual void handleNocRequest(PacketPtr pkt) = 0;

    virtual void handleCpuRequest(PacketPtr pkt) = 0;

};

#endif // __MEM_DTU_BASE_HH__
