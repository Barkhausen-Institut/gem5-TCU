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

#include "mem/dtu/base.hh"
#include "mem/dtu/regfile.hh"
#include "mem/dtu/noc_addr.hh"
#include "mem/dtu/pt_unit.hh"
#include "params/Dtu.hh"

class MessageUnit;
class MemoryUnit;
class XferUnit;

class Dtu : public BaseDtu
{
  public:

    static const uint16_t INVALID_VPE_ID    = 0xFFFF;
    static const size_t CREDITS_UNLIM       = 0xFFFF;

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

    enum class Error
    {
        NONE                = 0,
        MISS_CREDITS        = 1,
        NO_RING_SPACE       = 2,
        VPE_GONE            = 3,
        PAGEFAULT           = 4,
        NO_MAPPING          = 5,
    };

    struct MessageHeader
    {
         // if bit 0 is set its a reply, if bit 1 is set we grant credits
        uint8_t flags;
        uint8_t senderCoreId;
        uint8_t senderEpId;
         // for a normal message this is the reply epId
        // for a reply this is the enpoint that receives credits
        uint8_t replyEpId;
        uint16_t length;
        uint16_t senderVpeId;

        // both should be large enough for pointers.
        uint64_t label;
        uint64_t replyLabel;
    } M5_ATTR_PACKED;

    enum class NocPacketType
    {
        MESSAGE,
        PAGEFAULT,
        READ_REQ,
        WRITE_REQ,
        CACHE_MEM_REQ_FUNC,
        CACHE_MEM_REQ,
    };

    enum class TransferType
    {
        // we are reading stuff out of our local memory and send it
        LOCAL_READ,
        // we received the read resp. from somebody and write it to local mem
        LOCAL_WRITE,
        // we received something and write it to our local memory
        REMOTE_WRITE,
        // we should send something from our local memory to somebody else
        REMOTE_READ
    };

    enum class MemReqType
    {
        TRANSFER,
        HEADER,
        TRANSLATION,
    };

    struct MemSenderState : public Packet::SenderState
    {
        Addr data;
        MasterID mid;
        MemReqType type;
    };

    struct NocSenderState : public Packet::SenderState
    {
        Error result;
        uint vpeId;
        NocPacketType packetType;
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
            INC_READ_PTR    = 5,
            DEBUG_MSG       = 6,
        };

        enum Flags
        {
            NONE            = 0,
            NOPF            = 1,
        };

        Error error;
        Opcode opcode;
        unsigned arg;
        unsigned flags;
    };

    struct ExternCommand
    {
        enum Opcode
        {
            WAKEUP_CORE     = 0,
            INV_PAGE        = 1,
            INV_TLB         = 2,
            INV_CACHE       = 3,
            INJECT_IRQ      = 4,
        };

        Opcode opcode;
        uint64_t arg;
    };

  public:

    static constexpr unsigned numCmdOpcodeBits = 3;

  public:

    Dtu(DtuParams* p);

    ~Dtu();

    void regStats() override;

    RegFile &regs() { return regFile; }

    PacketPtr generateRequest(Addr addr, Addr size, MemCmd cmd);
    void freeRequest(PacketPtr pkt);

    void wakeupCore();

    void updateSuspendablePin();

    void injectIRQ(int vector);

    void forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest);

    void sendFunctionalMemRequest(PacketPtr pkt)
    {
        dcacheMasterPort.sendFunctional(pkt);
    }

    void scheduleFinishOp(Cycles delay, Error error = Error::NONE)
    {
        if (cmdInProgress)
            schedule(new FinishCommandEvent(*this, error), clockEdge(delay));
    }

    void scheduleCommand(Cycles delay)
    {
        schedule(executeCommandEvent, clockEdge(delay));
    }

    void sendMemRequest(PacketPtr pkt,
                        Addr virt,
                        Addr data,
                        MemReqType type,
                        Cycles delay);

    void sendNocRequest(NocPacketType type,
                        PacketPtr pkt,
                        uint vpeId,
                        uint flags,
                        Cycles delay,
                        bool functional = false);

    void sendNocResponse(PacketPtr pkt);

    void startTransfer(TransferType type,
                       NocAddr targetAddr,
                       Addr sourceAddr,
                       Addr size,
                       PacketPtr pkt = NULL,
                       uint vpeId = 0,
                       MessageHeader* header = NULL,
                       Cycles delay = Cycles(0),
                       uint flags = 0);

    void startTranslate(Addr virt,
                        uint access,
                        PtUnit::Translation *trans);

    void finishMsgReceive(unsigned epId);

    void handlePFResp(PacketPtr pkt);

    void printPacket(PacketPtr pkt) const;

  private:

    Command getCommand();

    void executeCommand();

    ExternCommand getExternCommand();

    void executeExternCommand(PacketPtr pkt);

    void finishCommand(Error error);

    void completeNocRequest(PacketPtr pkt) override;

    void completeMemRequest(PacketPtr pkt) override;

    void handleNocRequest(PacketPtr pkt) override;

    bool handleCpuRequest(PacketPtr pkt,
                          DtuSlavePort &sport,
                          DtuMasterPort &mport,
                          bool icache,
                          bool functional) override;

    bool handleCacheMemRequest(PacketPtr pkt, bool functional) override;

    int translate(PtUnit::Translation *trans,
                  PacketPtr pkt,
                  bool icache,
                  bool functional) override;

  private:

    const MasterID masterId;

    System *system;

    RegFile regFile;

    MessageUnit *msgUnit;

    MemoryUnit *memUnit;

    XferUnit *xferUnit;

    PtUnit *ptUnit;

    EventWrapper<Dtu, &Dtu::executeCommand> executeCommandEvent;

    struct ExecExternCmdEvent : public Event
    {
        Dtu& dtu;

        PacketPtr pkt;

        ExecExternCmdEvent(Dtu& _dtu, PacketPtr _pkt)
            : dtu(_dtu), pkt(_pkt)
        {}

        void process() override
        {
            dtu.executeExternCommand(pkt);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "ExecExternCmdEvent"; }

        const std::string name() const override { return dtu.name(); }
    };

    struct FinishCommandEvent : public Event
    {
        Dtu& dtu;

        Error error;

        FinishCommandEvent(Dtu& _dtu, Error _error = Error::NONE)
            : dtu(_dtu), error(_error)
        {}

        void process() override
        {
            dtu.finishCommand(error);
            setFlags(AutoDelete);
        }

        const char* description() const override { return "FinishCommandEvent"; }

        const std::string name() const override { return dtu.name(); }
    };

    struct MemTranslation : PtUnit::Translation
    {
        Dtu& dtu;

        DtuSlavePort& sport;
        DtuMasterPort& mport;

        PacketPtr pkt;

        MemTranslation(Dtu &_dtu,
                       DtuSlavePort& _sport,
                       DtuMasterPort& _mport,
                       PacketPtr _pkt)
            : dtu(_dtu), sport(_sport), mport(_mport), pkt(_pkt)
        {}

        void finished(bool success, const NocAddr &phys) override
        {
            if (!success)
                dtu.sendDummyResponse(sport, pkt, false);
            else
            {
                pkt->setAddr(phys.getAddr());
                pkt->req->setPaddr(phys.getAddr());

                mport.schedTimingReq(pkt, curTick());
            }

            delete this;
        }
    };

    struct VPEGoneTranslation : PtUnit::Translation
    {
        Dtu& dtu;

        PacketPtr pkt;

        VPEGoneTranslation(Dtu& _dtu, PacketPtr _pkt)
            : dtu(_dtu), pkt(_pkt)
        {}

        void finished(bool success, const NocAddr &phys) override
        {
            if (success)
                dtu.handleCacheMemRequest(pkt, false);
            else
                dtu.sendCacheMemResponse(pkt, false);

            delete this;
        }
    };

    bool cmdInProgress;

  public:

    DtuTlb *tlb;

    unsigned memPe;
    Addr memOffset;

    const bool atomicMode;

    const unsigned numEndpoints;

    const Addr maxNocPacketSize;

    const unsigned numCmdArgBits;
    const unsigned numCmdFlagsBits;

    const size_t blockSize;

    const size_t bufCount;
    const size_t bufSize;

    const unsigned cacheBlocksPerCycle;

    const Cycles registerAccessLatency;

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

    // commands
    Stats::Vector commands;
    Stats::Vector extCommands;

};

#endif // __MEM_DTU_DTU_HH__
