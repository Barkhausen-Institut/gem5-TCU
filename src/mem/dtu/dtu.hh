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

#ifndef __MEM_DTU_DTU_HH__
#define __MEM_DTU_DTU_HH__

#include "mem/dtu/connector/base.hh"
#include "mem/dtu/base.hh"
#include "mem/dtu/regfile.hh"
#include "mem/dtu/noc_addr.hh"
#include "mem/dtu/pt_unit.hh"
#include "mem/dtu/xfer_unit.hh"
#include "mem/dtu/core_reqs.hh"
#include "mem/dtu/error.hh"
#include "params/Dtu.hh"

class MessageUnit;
class MemoryUnit;
class XferUnit;

class Dtu : public BaseDtu
{
  public:

    static const uint16_t INVALID_VPE_ID    = 0xFFFF;
    static const size_t CREDITS_UNLIM       = 0x3F;

    static const int SYSCALL_EP             = 0;

    enum MemoryFlags : uint8_t
    {
        READ                = (1 << 0),
        WRITE               = (1 << 1),
    };

    enum MessageFlags : uint8_t
    {
        REPLY_FLAG          = (1 << 0),
        GRANT_CREDITS_FLAG  = (1 << 1),
        REPLY_ENABLED       = (1 << 2),
        PAGEFAULT           = (1 << 3),
    };

    enum class NocPacketType
    {
        MESSAGE,
        PAGEFAULT,
        READ_REQ,
        WRITE_REQ,
        CACHE_MEM_REQ_FUNC,
        CACHE_MEM_REQ,
    };

    enum class MemReqType
    {
        TRANSFER,
        TRANSLATION,
    };

    struct MemSenderState : public Packet::SenderState
    {
        Addr data;
        MasterID mid;
        MemReqType type;
    };

    enum NocFlags
    {
        NONE    = 0,
        NOPF    = 1,
        PRIV    = 2,
    };

    struct NocSenderState : public Packet::SenderState
    {
        DtuError result;
        NocPacketType packetType;
        uint64_t cmdId;
        uint flags;
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
            FETCH_EVENTS    = 6,
            SET_EVENT       = 7,
            ACK_MSG         = 8,
            SLEEP           = 9,
            PRINT           = 10,
        };

        enum Flags
        {
            NONE            = 0,
            NOPF            = 1,
        };

        BitUnion64(Bits)
            Bitfield<63, 17> arg;
            Bitfield<16, 13> error;
            Bitfield<12> flags;
            Bitfield<11, 4> epid;
            Bitfield<3, 0> opcode;
        EndBitUnion(Bits)

        Bits value;
    };

    struct PrivCommand
    {
        enum Opcode
        {
            IDLE            = 0,
            INV_EP          = 1,
            INV_PAGE        = 2,
            INV_TLB         = 3,
            INV_REPLY       = 4,
            RESET           = 5,
            FLUSH_CACHE     = 6,
            XCHG_VPE        = 7,
        };

        Opcode opcode;
        uint64_t arg;
    };

  public:

    Dtu(DtuParams* p);

    ~Dtu();

    void regStats() override;

    RegFile &regs() { return regFile; }

    DtuTlb *tlb() { return tlBuf; }

    bool isMemPE(unsigned pe) const;

    PacketPtr generateRequest(Addr addr, Addr size, MemCmd cmd);
    void freeRequest(PacketPtr pkt);

    void printLine(Addr len);

    bool startSleep(uint64_t cycles, bool ack);

    void stopSleep();

    void wakeupCore();

    Cycles reset(Addr entry, bool flushInval);

    Cycles flushInvalCaches(bool invalidate);

    void setIrq();

    void clearIrq();

    void forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest);

    void sendFunctionalMemRequest(PacketPtr pkt)
    {
        // set our master id (it might be from a different PE)
        pkt->req->setMasterId(masterId);

        dcacheMasterPort.sendFunctional(pkt);
    }

    void scheduleFinishOp(Cycles delay, DtuError error = DtuError::NONE);

    void sendMemRequest(PacketPtr pkt,
                        Addr virt,
                        Addr data,
                        MemReqType type,
                        Cycles delay);

    void sendNocRequest(NocPacketType type,
                        PacketPtr pkt,
                        uint flags,
                        Cycles delay,
                        bool functional = false);

    void sendNocResponse(PacketPtr pkt);

    void setCommandSent() { cmdSent = true; }

    Addr physToNoc(Addr phys) const;
    Addr nocToPhys(Addr noc) const;

    void startTransfer(void *event, Cycles delay);

    void startTranslate(size_t id,
                        Addr virt,
                        uint access,
                        PtUnit::Translation *trans);

    void startForeignReceive(size_t id,
                             unsigned epId,
                             unsigned vpeId,
                             XferUnit::TransferEvent *event);

    void abortTranslate(size_t id, PtUnit::Translation *trans);

    void handlePFResp(PacketPtr pkt);

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

    void finishCommand(DtuError error);

    void completeNocRequest(PacketPtr pkt) override;

    void completeMemRequest(PacketPtr pkt) override;

    void handleNocRequest(PacketPtr pkt) override;

    bool handleCoreMemRequest(PacketPtr pkt,
                              DtuSlavePort &sport,
                              DtuMasterPort &mport,
                              bool icache,
                              bool functional) override;

    void completeCoreMemReqs();

    bool handleCacheMemRequest(PacketPtr pkt, bool functional) override;

    int translate(PtUnit::Translation *trans,
                  PacketPtr pkt,
                  bool icache,
                  bool functional) override;

  private:

    const MasterID masterId;

    System *system;

    RegFile regFile;

    BaseConnector *connector;

    DtuTlb *tlBuf;

    MessageUnit *msgUnit;

    MemoryUnit *memUnit;

    XferUnit *xferUnit;

    PtUnit *ptUnit;

    CoreRequests coreReqs;

    EventWrapper<Dtu, &Dtu::abortCommand> abortCommandEvent;

    EventWrapper<CoreRequests, &CoreRequests::completeReqs> completeCoreReqEvent;

    struct DtuEvent : public Event
    {
        Dtu& dtu;

        DtuEvent(Dtu& _dtu)
            : dtu(_dtu)
        {}

        const std::string name() const override { return dtu.name(); }
    };

    struct ExecCmdEvent : public DtuEvent
    {
        PacketPtr pkt;

        ExecCmdEvent(Dtu& _dtu, PacketPtr _pkt)
            : DtuEvent(_dtu), pkt(_pkt)
        {}

        void process() override
        {
            dtu.executeCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecCmdEvent"; }
    };

    struct ExecPrivCmdEvent : public DtuEvent
    {
        PacketPtr pkt;

        ExecPrivCmdEvent(Dtu& _dtu, PacketPtr _pkt)
            : DtuEvent(_dtu), pkt(_pkt)
        {}

        void process() override
        {
            dtu.executePrivCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecPrivCmdEvent"; }
    };

    struct FinishCommandEvent : public DtuEvent
    {
        DtuError error;

        FinishCommandEvent(Dtu& _dtu, DtuError _error = DtuError::NONE)
            : DtuEvent(_dtu), error(_error)
        {}

        void process() override
        {
            dtu.finishCommand(error);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "FinishCommandEvent"; }
    };

    struct MemTranslation : PtUnit::Translation
    {
        Dtu& dtu;

        DtuSlavePort& sport;
        DtuMasterPort& mport;

        PacketPtr pkt;

        bool complete;
        bool success;
        NocAddr phys;

        MemTranslation(Dtu &_dtu,
                       DtuSlavePort& _sport,
                       DtuMasterPort& _mport,
                       PacketPtr _pkt)
            : dtu(_dtu), sport(_sport), mport(_mport), pkt(_pkt),
              complete(), success(), phys()
        {}

        void finished(bool success, const NocAddr &phys) override;
    };

    PacketPtr cmdPkt;
    FinishCommandEvent *cmdFinish;
    uint64_t cmdId;
    uint abortCmd;
    size_t cmdXferBuf;
    bool cmdSent;

    std::list<MemTranslation*> xlates;

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

    static uint64_t nextCmdId;

};

#endif // __MEM_DTU_DTU_HH__
