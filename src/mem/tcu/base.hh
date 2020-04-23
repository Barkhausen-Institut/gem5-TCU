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

#ifndef __MEM_TCU_BASE_HH__
#define __MEM_TCU_BASE_HH__

#include <queue>

#include "mem/mem_object.hh"
#include "mem/qport.hh"
#include "params/BaseTcu.hh"
#include "mem/tcu/tlb.hh"

class BaseTcu : public ClockedObject
{
  protected:

    class TcuMasterPort : public QueuedMasterPort
    {
      protected:

        BaseTcu& tcu;

        ReqPacketQueue reqQueue;

        SnoopRespPacketQueue snoopRespQueue;

      public:

        TcuMasterPort( const std::string& _name, BaseTcu& _tcu);

        virtual void completeRequest(PacketPtr pkt) = 0;

        bool recvTimingResp(PacketPtr pkt) override;
    };

    class NocMasterPort : public TcuMasterPort
    {
      public:

        NocMasterPort(BaseTcu& _tcu)
            : TcuMasterPort(_tcu.name() + ".noc_master_port", _tcu)
        { }

        void completeRequest(PacketPtr pkt) override;
    };

    class ICacheMasterPort : public TcuMasterPort
    {
      public:

        ICacheMasterPort(BaseTcu& _tcu)
          : TcuMasterPort(_tcu.name() + ".icache_master_port", _tcu)
        { }

        void completeRequest(PacketPtr) override {
        }
        bool recvTimingResp(PacketPtr pkt) override;
    };

    class DCacheMasterPort : public TcuMasterPort
    {
      public:

        DCacheMasterPort(BaseTcu& _tcu)
          : TcuMasterPort(_tcu.name() + ".dcache_master_port", _tcu)
        { }

        void completeRequest(PacketPtr) override {
        }
        bool recvTimingResp(PacketPtr pkt) override;
    };

    class TcuSlavePort : public SlavePort
    {
      protected:

        BaseTcu& tcu;

        bool busy;

        bool sendReqRetry;

        struct ResponseEvent : public Event
        {
            TcuSlavePort& port;

            PacketPtr pkt;

            ResponseEvent(TcuSlavePort& _port, PacketPtr _pkt)
                : port(_port), pkt(_pkt) {}

            void process() override;

            const char* description() const override
            {
                return "TCU ResponseEvent";
            }

            const std::string name() const override { return port.name(); }
        };

        std::queue<ResponseEvent*> pendingResponses;

      public:

        TcuSlavePort(const std::string& _name, BaseTcu& _tcu);

        virtual bool handleRequest(PacketPtr pkt,
                                   bool *busy,
                                   bool functional) = 0;

        void schedTimingResp(PacketPtr pkt, Tick when);

        Tick recvAtomic(PacketPtr pkt) override;

        void recvFunctional(PacketPtr pkt) override;

        bool recvTimingReq(PacketPtr pkt) override;

        void recvRespRetry() override;

        void requestFinished();
    };

    class NocSlavePort : public TcuSlavePort
    {
      public:

        NocSlavePort(BaseTcu& _tcu)
          : TcuSlavePort(_tcu.name() + ".noc_slave_port", _tcu)
        { }

      protected:

        AddrRangeList getAddrRanges() const override;

        bool handleRequest(PacketPtr pkt,
                           bool *busy,
                           bool functional) override;
    };

    template<class T>
    class CacheSlavePort : public TcuSlavePort
    {
      private:

        T &port;

        bool icache;

      public:

        CacheSlavePort(T &_port, BaseTcu& _tcu, bool _icache)
          : TcuSlavePort(_tcu.name() + (_icache ? ".icache_slave_port" : ".dcache_slave_port"), _tcu),
            port(_port), icache(_icache)
        { }

        AddrRangeList getAddrRanges() const override
        {
            AddrRangeList ranges;

            for (auto &r : tcu.slaveRegion)
                ranges.push_back(r);

            return ranges;
        }

        bool handleRequest(PacketPtr pkt, bool *, bool functional) override
        {
            bool res = tcu.handleCoreMemRequest(pkt, *this, port, icache, functional);
            if (!res)
                tcu.sendDummyResponse(*this, pkt, functional);
            return true;
        }
    };

    class CacheMemSlavePort : public TcuSlavePort
    {
        friend class NocMasterPort;

      public:

        CacheMemSlavePort(BaseTcu& _tcu)
          : TcuSlavePort(_tcu.name() + ".cache_mem_slave_port", _tcu)
        { }

      protected:

        AddrRangeList getAddrRanges() const override;

        bool handleRequest(PacketPtr pkt,
                           bool *busy,
                           bool functional) override;
    };

  public:

    BaseTcu(BaseTcuParams* p);

    void init() override;

    Port& getPort(const std::string &n, PortID idx) override;

    void schedNocResponse(PacketPtr pkt, Tick when);

    void schedCpuResponse(PacketPtr pkt, Tick when);

    void schedNocRequest(PacketPtr pkt, Tick when);

    void schedMemRequest(PacketPtr pkt, Tick when);

    void schedNocRequestFinished(Tick when);

    void sendFunctionalNocRequest(PacketPtr pkt);

    void sendAtomicNocRequest(PacketPtr pkt);

    void sendAtomicMemRequest(PacketPtr pkt);

    void sendCacheMemResponse(PacketPtr pkt, bool success);

    virtual void completeNocRequest(PacketPtr pkt) = 0;

    virtual void completeMemRequest(PacketPtr pkt) = 0;

    virtual void handleNocRequest(PacketPtr pkt) = 0;

    virtual bool handleCoreMemRequest(PacketPtr pkt,
                                      TcuSlavePort &sport,
                                      TcuMasterPort &mport,
                                      bool icache,
                                      bool functional) = 0;

    virtual bool handleCacheMemRequest(PacketPtr pkt, bool functional) = 0;

  protected:

    void nocRequestFinished();

    void sendDummyResponse(TcuSlavePort &port, PacketPtr pkt, bool functional);

    void printNocRequest(PacketPtr pkt, const char *type);

    NocMasterPort  nocMasterPort;

    NocSlavePort   nocSlavePort;

    ICacheMasterPort icacheMasterPort;

    DCacheMasterPort dcacheMasterPort;

    CacheSlavePort<ICacheMasterPort> icacheSlavePort;

    CacheSlavePort<DCacheMasterPort> dcacheSlavePort;

    CacheMemSlavePort cacheMemSlavePort;

    std::vector<Cache*> caches;

    EventWrapper<BaseTcu, &BaseTcu::nocRequestFinished> nocReqFinishedEvent;

  public:

    const peid_t peId;

    const AddrRange mmioRegion;

    const std::vector<AddrRange> slaveRegion;

};

#endif // __MEM_TCU_BASE_HH__
