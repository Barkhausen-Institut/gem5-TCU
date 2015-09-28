/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (c) 2015, Nils Asmussen
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

    class ICacheMasterPort : public DtuMasterPort
    {
      public:

        ICacheMasterPort(BaseDtu& _dtu)
          : DtuMasterPort(_dtu.name() + ".icache_master_port", _dtu)
        { }

        void completeRequest(PacketPtr) override {
        }
        bool recvTimingResp(PacketPtr pkt) override;
    };

    class DCacheMasterPort : public DtuMasterPort
    {
      public:

        DCacheMasterPort(BaseDtu& _dtu)
          : DtuMasterPort(_dtu.name() + ".dcache_master_port", _dtu)
        { }

        void completeRequest(PacketPtr) override {
        }
        bool recvTimingResp(PacketPtr pkt) override;
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

        DtuSlavePort(const std::string& _name, BaseDtu& _dtu);

        virtual bool handleRequest(PacketPtr pkt, bool *busy, bool functional) = 0;

        void schedTimingResp(PacketPtr pkt, Tick when);

        Tick recvAtomic(PacketPtr pkt) override;

        void recvFunctional(PacketPtr pkt) override;

        bool recvTimingReq(PacketPtr pkt) override;

        void recvRespRetry() override;
    };

    class NocSlavePort : public DtuSlavePort
    {
      public:

        NocSlavePort(BaseDtu& _dtu)
          : DtuSlavePort(_dtu.name() + ".noc_slave_port", _dtu)
        { }

      protected:

        AddrRangeList getAddrRanges() const override;

        bool handleRequest(PacketPtr pkt, bool *busy, bool functional) override;
    };

    template<class T>
    class CacheSlavePort : public DtuSlavePort
    {
      private:
        
        T &port;

      public:

        CacheSlavePort(T &_port, BaseDtu& _dtu)
          : DtuSlavePort(_dtu.name() + ".icache_slave_port", _dtu),
            port(_port)
        { }

        AddrRangeList getAddrRanges() const override
        {
            AddrRangeList ranges;

            auto range = AddrRange(0, static_cast<Addr>(-1));

            ranges.push_back(range);

            return ranges;
        }

        bool handleRequest(PacketPtr pkt, bool *busy, bool functional) override
        {
            if(pkt->getAddr() >= dtu.regFileBaseAddr)
            {
                // not supported here
                assert(!functional);

                // set that before handling the request, because it will schedule the response
                *busy = true;
                dtu.handleCpuRequest(pkt);
            }
            else if(AddrRange(pkt->getAddr(),
                pkt->getAddr() + (pkt->getSize() - 1)).isSubset(AddrRange(0, 8 * 1024 * 1024 - 1)))
            {
                if(functional)
                    port.sendFunctional(pkt);
                else
                    port.schedTimingReq(pkt, curTick());
            }
            // report an error for other requests (due to speculative execution, there may be invalid
            // requests that will simply be ignored)
            else
                return false;
            return true;
        }
    };

  protected:

    NocMasterPort  nocMasterPort;

    NocSlavePort   nocSlavePort;

    ICacheMasterPort icacheMasterPort;

    DCacheMasterPort dcacheMasterPort;

    CacheSlavePort<ICacheMasterPort> icacheSlavePort;

    CacheSlavePort<DCacheMasterPort> dcacheSlavePort;

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
