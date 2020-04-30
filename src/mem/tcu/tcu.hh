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

#ifndef __MEM_TCU_TCU_HH__
#define __MEM_TCU_TCU_HH__

#include "mem/tcu/connector/base.hh"
#include "mem/tcu/base.hh"
#include "mem/tcu/regfile.hh"
#include "mem/tcu/noc_addr.hh"
#include "mem/tcu/xfer_unit.hh"
#include "mem/tcu/core_reqs.hh"
#include "mem/tcu/error.hh"
#include "params/Tcu.hh"

class MessageUnit;
class MemoryUnit;
class XferUnit;

class Tcu : public BaseTcu
{
  public:

    static const uint16_t INVALID_VPE_ID    = 0xFFFF;
    static const size_t CREDITS_UNLIM       = 0x3F;
    static const uint16_t INVALID_EP_ID     = 0xFFFF;

    enum MemoryFlags : uint8_t
    {
        READ                = (1 << 0),
        WRITE               = (1 << 1),
    };

    enum MessageFlags : uint8_t
    {
        REPLY_FLAG          = (1 << 0),
    };

    enum class AbortType
    {
        NONE,
        LOCAL,
        REMOTE,
    };

    enum class NocPacketType
    {
        MESSAGE,
        READ_REQ,
        WRITE_REQ,
        CACHE_MEM_REQ_FUNC,
        CACHE_MEM_REQ,
    };

    struct MemSenderState : public Packet::SenderState
    {
        Addr data;
        MasterID mid;
    };

    struct NocSenderState : public Packet::SenderState
    {
        TcuError result;
        NocPacketType packetType;
        vpeid_t tvpe;
    };

    struct InitSenderState : public Packet::SenderState
    {
    };

    struct Command
    {
        enum Opcode
        {
            IDLE            = 0,
            SEND            = 1,
            REPLY           = 2,
            READ            = 3,
            WRITE           = 4,
            FETCH_MSG       = 5,
            ACK_MSG         = 6,
            SLEEP           = 7,
        };

        BitUnion64(Bits)
            Bitfield<56, 24> arg;
            Bitfield<23, 20> error;
            Bitfield<19, 4> epid;
            Bitfield<3, 0> opcode;
        EndBitUnion(Bits)

        Bits value;
    };

    struct PrivCommand
    {
        enum Opcode
        {
            IDLE            = 0,
            INV_PAGE        = 1,
            INV_TLB         = 2,
            INS_TLB         = 3,
            XCHG_VPE        = 4,
            FLUSH_CACHE     = 5,
            SET_TIMER       = 6,
            ABORT_CMD       = 7,
        };

        Opcode opcode;
        uint64_t arg;
    };

    struct ExtCommand
    {
        enum Opcode
        {
            IDLE            = 0,
            INV_EP          = 1,
            INV_REPLY       = 2,
            RESET           = 3,
        };

        Opcode opcode;
        TcuError error;
        uint64_t arg;
    };

  public:

    Tcu(TcuParams* p);

    ~Tcu();

    void regStats() override;

    RegFile &regs() { return regFile; }

    TcuTlb *tlb() { return tlBuf; }

    bool isMemPE(unsigned pe) const;

    PacketPtr generateRequest(Addr addr, Addr size, MemCmd cmd);
    void freeRequest(PacketPtr pkt);

    void printLine(Addr len);

    bool startSleep(epid_t wakeupEp);

    void stopSleep();

    void wakeupCore(bool force);

    Cycles reset(bool flushInval);

    Cycles flushInvalCaches(bool invalidate);

    void setIrq(BaseConnector::IRQ irq);

    void clearIrq(BaseConnector::IRQ irq);

    void fireTimer();

    void forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest);

    void sendFunctionalMemRequest(PacketPtr pkt)
    {
        // set our master id (it might be from a different PE)
        pkt->req->setMasterId(masterId);

        dcacheMasterPort.sendFunctional(pkt);
    }

    void scheduleFinishOp(Cycles delay, TcuError error = TcuError::NONE);

    void sendMemRequest(PacketPtr pkt,
                        Addr virt,
                        Addr data,
                        Cycles delay);

    void sendNocRequest(NocPacketType type,
                        PacketPtr pkt,
                        vpeid_t tvpe,
                        Cycles delay,
                        bool functional = false);

    void sendNocResponse(PacketPtr pkt);

    static Addr physToNoc(Addr phys);
    static Addr nocToPhys(Addr noc);

    void startTransfer(void *event, Cycles delay);

    size_t startTranslate(vpeid_t vpeId,
                          Addr virt,
                          uint access,
                          bool can_pf,
                          XferUnit::Translation *trans);

    size_t startForeignReceive(epid_t epId, vpeid_t vpeId);

    void abortTranslate(size_t reqId);

    bool isCommandAborting() const
    {
        return abort != AbortType::NONE;
    }

    void printPacket(PacketPtr pkt) const;

  private:

    Command::Bits getCommand()
    {
        return regFile.get(CmdReg::COMMAND);
    }

    void executeCommand(PacketPtr pkt);

    void abortCommand();

    PrivCommand getPrivCommand();

    void executePrivCommand(PacketPtr pkt);

    void finishAbort();

    ExtCommand getExtCommand();

    void executeExtCommand(PacketPtr pkt);

    void finishCommand(TcuError error);

    bool has_message(epid_t ep);

    void completeNocRequest(PacketPtr pkt) override;

    void completeMemRequest(PacketPtr pkt) override;

    void handleNocRequest(PacketPtr pkt) override;

    bool handleCoreMemRequest(PacketPtr pkt,
                              TcuSlavePort &sport,
                              TcuMasterPort &mport,
                              bool icache,
                              bool functional) override;

    bool handleCacheMemRequest(PacketPtr pkt, bool functional) override;

  private:

    const MasterID masterId;

    System *system;

    RegFile regFile;

    BaseConnector *connector;

    TcuTlb *tlBuf;

    MessageUnit *msgUnit;

    MemoryUnit *memUnit;

    XferUnit *xferUnit;

    CoreRequests coreReqs;

    EventWrapper<Tcu, &Tcu::fireTimer> fireTimerEvent;

    EventWrapper<CoreRequests, &CoreRequests::completeReqs> completeCoreReqEvent;

    struct TcuEvent : public Event
    {
        Tcu& tcu;

        TcuEvent(Tcu& _tcu)
            : tcu(_tcu)
        {}

        const std::string name() const override { return tcu.name(); }
    };

    struct ExecCmdEvent : public TcuEvent
    {
        PacketPtr pkt;

        ExecCmdEvent(Tcu& _tcu, PacketPtr _pkt)
            : TcuEvent(_tcu), pkt(_pkt)
        {}

        void process() override
        {
            tcu.executeCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecCmdEvent"; }
    };

    struct ExecPrivCmdEvent : public TcuEvent
    {
        PacketPtr pkt;

        ExecPrivCmdEvent(Tcu& _tcu, PacketPtr _pkt)
            : TcuEvent(_tcu), pkt(_pkt)
        {}

        void process() override
        {
            tcu.executePrivCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecPrivCmdEvent"; }
    };

    struct ExecExtCmdEvent : public TcuEvent
    {
        PacketPtr pkt;

        ExecExtCmdEvent(Tcu& _tcu, PacketPtr _pkt)
            : TcuEvent(_tcu), pkt(_pkt)
        {}

        void process() override
        {
            tcu.executeExtCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecExtCmdEvent"; }
    };

    struct FinishCommandEvent : public TcuEvent
    {
        TcuError error;

        FinishCommandEvent(Tcu& _tcu, TcuError _error = TcuError::NONE)
            : TcuEvent(_tcu), error(_error)
        {}

        void process() override
        {
            tcu.finishCommand(error);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "FinishCommandEvent"; }
    };

    PacketPtr cmdPkt;
    PacketPtr privCmdPkt;
    FinishCommandEvent *cmdFinish;
    AbortType abort;
    bool cmdIsRemote;
    int wakeupEp;

  public:

    unsigned memPe;
    Addr memOffset;
    Addr memSize;

    const bool atomicMode;

    const unsigned numEndpoints;

    const Addr maxNocPacketSize;

    const size_t blockSize;

    const size_t bufCount;
    const size_t bufSize;
    const size_t reqCount;

    const unsigned cacheBlocksPerCycle;

    const Cycles registerAccessLatency;

    const Cycles cpuToCacheLatency;

    const Cycles commandToNocRequestLatency;
    const Cycles startMsgTransferDelay;

    const Cycles transferToMemRequestLatency;
    const Cycles transferToNocLatency;
    const Cycles nocToTransferLatency;

    // NoC receives
    Stats::Scalar nocMsgRecvs;
    Stats::Scalar nocReadRecvs;
    Stats::Scalar nocWriteRecvs;

    // other
    Stats::Scalar regFileReqs;
    Stats::Scalar intMemReqs;
    Stats::Scalar extMemReqs;
    Stats::Scalar irqInjects;
    Stats::Scalar resets;

    // commands
    Stats::Vector commands;
    Stats::Vector privCommands;
    Stats::Vector extCommands;

};

#endif // __MEM_TCU_TCU_HH__
