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

#include <queue>

#include "mem/mem_object.hh"
#include "mem/qport.hh"
#include "params/BaseDtu.hh"
#include "mem/dtu/tlb.hh"
#include "mem/dtu/pt_unit.hh"

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

        struct ResponseEvent : public Event
        {
            DtuSlavePort& port;

            PacketPtr pkt;

            ResponseEvent(DtuSlavePort& _port, PacketPtr _pkt)
                : port(_port), pkt(_pkt) {}

            void process() override;

            const char* description() const override
            {
                return "DTU ResponseEvent";
            }

            const std::string name() const override { return port.name(); }
        };

        std::queue<ResponseEvent*> pendingResponses;

      public:

        DtuSlavePort(const std::string& _name, BaseDtu& _dtu);

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

    class NocSlavePort : public DtuSlavePort
    {
      public:

        NocSlavePort(BaseDtu& _dtu)
          : DtuSlavePort(_dtu.name() + ".noc_slave_port", _dtu)
        { }

      protected:

        AddrRangeList getAddrRanges() const override;

        bool handleRequest(PacketPtr pkt,
                           bool *busy,
                           bool functional) override;
    };

    template<class T>
    class CacheSlavePort : public DtuSlavePort
    {
      private:

        struct Translation : PtUnit::Translation
        {
            CacheSlavePort& base;

            PacketPtr pkt;

            Translation(CacheSlavePort& _base, PacketPtr _pkt)
                : base(_base), pkt(_pkt)
            {}

            void finished(bool success, const NocAddr &phys) override
            {
                if (!success)
                    base.dtu.sendDummyResponse(base, pkt, false);
                else
                {
                    pkt->setAddr(phys.getAddr());
                    pkt->req->setPaddr(phys.getAddr());

                    base.port.schedTimingReq(pkt, curTick());
                }

                delete this;
            }
        };

        T &port;

        bool icache;

      public:

        CacheSlavePort(T &_port, BaseDtu& _dtu, bool _icache)
          : DtuSlavePort(_dtu.name() + ".icache_slave_port", _dtu),
            port(_port), icache(_icache)
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
            if (pkt->getAddr() >= dtu.regFileBaseAddr)
            {
                // not supported here
                assert(!functional);

                dtu.handleCpuRequest(pkt);
            }
            else
            {
                dtu.checkWatchRange(pkt);

                Translation *trans = new Translation(*this, pkt);
                if (dtu.translate(trans, pkt, icache, functional))
                {
                    if (functional)
                        port.sendFunctional(pkt);
                    else
                        port.schedTimingReq(pkt, curTick());
                    delete trans;
                }
            }
            return true;
        }
    };

    class CacheMemSlavePort : public DtuSlavePort
    {
      public:

        CacheMemSlavePort(BaseDtu& _dtu)
          : DtuSlavePort(_dtu.name() + ".cache_mem_slave_port", _dtu)
        { }

      protected:

        AddrRangeList getAddrRanges() const override;

        bool handleRequest(PacketPtr pkt,
                           bool *busy,
                           bool functional) override;
    };

  public:

    BaseDtu(BaseDtuParams* p);

    void init() override;

    BaseSlavePort& getSlavePort(const std::string &n, PortID idx) override;

    BaseMasterPort& getMasterPort(const std::string &n, PortID idx) override;

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

    virtual void handleCpuRequest(PacketPtr pkt) = 0;

    virtual bool handleCacheMemRequest(PacketPtr pkt, bool functional) = 0;

    virtual bool translate(PtUnit::Translation *trans,
                           PacketPtr pkt,
                           bool icache,
                           bool functional) = 0;

  protected:

    void nocRequestFinished();

    void checkWatchRange(PacketPtr pkt);

    void sendDummyResponse(DtuSlavePort &port, PacketPtr pkt, bool functional);

    NocMasterPort  nocMasterPort;

    NocSlavePort   nocSlavePort;

    ICacheMasterPort icacheMasterPort;

    DCacheMasterPort dcacheMasterPort;

    CacheSlavePort<ICacheMasterPort> icacheSlavePort;

    CacheSlavePort<DCacheMasterPort> dcacheSlavePort;

    CacheMemSlavePort cacheMemSlavePort;

    AddrRange watchRange;

    EventWrapper<BaseDtu, &BaseDtu::nocRequestFinished> nocReqFinishedEvent;

  public:

    const unsigned coreId;

    const Addr regFileBaseAddr;

};

#endif // __MEM_DTU_BASE_HH__
