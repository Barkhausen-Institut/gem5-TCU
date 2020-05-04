/*
 * Copyright (c) 2017, Georg Kotheimer
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

#ifndef __DEV_TCU_PCI_PROXY__
#define __DEV_TCU_PCI_PROXY__

#include "dev/tcu/pci_host.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/regfile.hh"
#include "mem/mem_object.hh"
#include "params/TcuPciProxy.hh"

class TcuPciProxy : public ClockedObject
{
  public:
    TcuPciProxy(const TcuPciProxyParams* p);

    void init() override;

    void setPciHost(TcuPciHost* pciHost) { this->pciHost = pciHost; }

    Port& getPort(const std::string& if_name,
                  PortID idx = InvalidPortID) override;

    void signalInterrupt();

  private:
    PacketPtr createPacket(Addr paddr, size_t size, MemCmd cmd);
    PacketPtr createPacket(
        Addr paddr, const void* data, size_t size, MemCmd cmd);
    static void freePacket(PacketPtr pkt);

    PacketPtr createTcuRegPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    static Addr getRegAddr(UnprivReg reg);
    PacketPtr createTcuCmdPkt(CmdCommand::Bits cmd, CmdData::Bits data,
        uint64_t arg1);
    static Addr encodePciAddress(PciBusAddr const& busAddr, Addr offset);
    PacketPtr createPciConfigPacket(PciBusAddr busAddr, Addr offset,
        const void* data, size_t size, MemCmd cmd);

    bool findDevice();

    void executeCommand(PacketPtr cmdPkt);
    void commandExecutionFinished();

    void sendInterruptCmd();
    void handleInterruptMessageContent(PacketPtr pkt);

    void forwardAccessToDeviceMem(PacketPtr pkt);
    void completeAccessToDeviceMem(PacketPtr pkt);

    bool handleDmaRequest(PacketPtr pkt);
    void sendDmaCmd();
    void handleDmaContent(PacketPtr pkt);

    void tick();

  private:
    class TcuMasterPort : public MasterPort
    {
      private:
        TcuPciProxy& pciProxy;
        ReqPacketQueue reqPacketQueue;

      public:
        TcuMasterPort(const std::string& _name, TcuPciProxy* _pciProxy)
            : MasterPort(_name, _pciProxy),
              pciProxy(*_pciProxy),
              reqPacketQueue(*_pciProxy, *this)
        {
        }

        void schedTimingReq(PacketPtr pkt, Tick when)
        {
            reqPacketQueue.schedSendTiming(pkt, when);
        }

        bool trySatisfyFunctional(PacketPtr pkt)
        {
            return reqPacketQueue.trySatisfyFunctional(pkt);
        }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override { reqPacketQueue.retry(); }
    };

    class TcuSlavePort : public QueuedSlavePort
    {
      private:
        TcuPciProxy& pciProxy;
        RespPacketQueue respPacketQueue;

      public:
        TcuSlavePort(const std::string& _name, TcuPciProxy* _pciProxy)
            : QueuedSlavePort(_name, _pciProxy, respPacketQueue),
              pciProxy(*_pciProxy),
              respPacketQueue(*_pciProxy, *this)
        {
        }

      protected:
        void recvFunctional(PacketPtr pkt) override;

        bool recvTimingReq(PacketPtr pkt) override;

        Tick recvAtomic(PacketPtr pkt) override;

        AddrRangeList getAddrRanges() const override;
    };

    class PioPort : public MasterPort
    {
      private:
        TcuPciProxy& pciProxy;
        ReqPacketQueue reqPacketQueue;

      public:
        PioPort(const std::string& _name, TcuPciProxy* _pciProxy)
            : MasterPort(_name, _pciProxy),
              pciProxy(*_pciProxy),
              reqPacketQueue(*_pciProxy, *this)
        {
        }

        void schedTimingReq(PacketPtr pkt, Tick when)
        {
            reqPacketQueue.schedSendTiming(pkt, when);
        }

        bool trySatisfyFunctional(PacketPtr pkt)
        {
            return reqPacketQueue.trySatisfyFunctional(pkt);
        }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override { reqPacketQueue.retry(); }
    };

    class DmaPort : public QueuedSlavePort
    {
      private:
        TcuPciProxy& pciProxy;
        RespPacketQueue respPacketQueue;

      public:
        DmaPort(const std::string& _name, TcuPciProxy* _pciProxy)
            : QueuedSlavePort(_name, _pciProxy, respPacketQueue),
              pciProxy(*_pciProxy),
              respPacketQueue(*_pciProxy, *this)
        {
        }

      protected:
        void recvFunctional(PacketPtr pkt) override;

        bool recvTimingReq(PacketPtr pkt) override;

        Tick recvAtomic(PacketPtr pkt) override;

        AddrRangeList getAddrRanges() const override;
    };

    struct TcuPciProxyEvent : public Event
    {
        TcuPciProxy& pciProxy;

        TcuPciProxyEvent(TcuPciProxy& _pciProxy) : pciProxy(_pciProxy) {}

        const std::string name() const override { return pciProxy.name(); }
    };

    class CommandSM
    {
      public:
        enum State
        {
            CMD_IDLE,
            CMD_SEND,
            CMD_WAIT
        };

        explicit CommandSM(TcuPciProxy* _pciProxy)
            : pciProxy(_pciProxy), state(CMD_IDLE), cmd(nullptr)
        {
        }

        const std::string name() const
        {
            return pciProxy->name() + "::CommandSM";
        }

        std::string stateName() const;

        bool isIdle() const { return state == CMD_IDLE; }

        void executeCommand(PacketPtr cmdPkt);

        void handleMemResp(PacketPtr pkt);

        void tick();

      private:
        TcuPciProxy* pciProxy;
        State state;
        PacketPtr cmd;
    };

  private:
    static const unsigned EP_INT;
    static const unsigned EP_DMA;
    static const Addr REG_ADDR;
    static const Addr INT_ADDR;
    static const Addr DMA_ADDR;

    TcuPciHost* pciHost;
    TcuMasterPort tcuMasterPort;
    TcuSlavePort tcuSlavePort;
    PioPort pioPort;
    DmaPort dmaPort;

    MasterID masterId;
    unsigned int id;
    Addr tcuRegBase;

    PciBusAddr deviceBusAddr;

    EventWrapper<TcuPciProxy, &TcuPciProxy::tick> tickEvent;
    CommandSM cmdSM;
    bool cmdRunning;
    bool interruptPending;
    PacketPtr pendingDmaReq;
    bool dmaRetry;
};

#endif // __DEV_TCU_PCI_PROXY__
