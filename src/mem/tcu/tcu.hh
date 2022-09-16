/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
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

#include "base/chunk_generator.hh"
#include "base/output.hh"
#include "mem/tcu/base.hh"
#include "mem/tcu/cmds.hh"
#include "mem/tcu/connector.hh"
#include "mem/tcu/ep_file.hh"
#include "mem/tcu/reg_file.hh"
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
    friend class TcuCommands;

  public:

    static const uint16_t INVALID_ACT_ID    = 0xFFFF;
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
        RequestorID mid;
        bool coverage;
    };

    struct NocSenderState : public Packet::SenderState
    {
        TcuError result;
        NocPacketType packetType;
    };

    struct InitSenderState : public Packet::SenderState
    {
        explicit InitSenderState(Addr _oldAddr) : oldAddr(_oldAddr) {}

        Addr oldAddr;
    };

    struct WriteCoverageEvent : public Event
    {
        Tcu &_tcu;
        uint16_t _act;
        uint8_t *_buffer;
        ChunkGenerator _gen;
        OutputStream *_out;
        std::ostream *_os;

        WriteCoverageEvent(Tcu& tcu, uint16_t act, Addr address, Addr size)
            : Event(),
              _tcu(tcu),
              _act(act),
              _buffer(new uint8_t[tcu.blockSize]),
              _gen(address, size, tcu.blockSize),
              _out(),
              _os()
        {}
        ~WriteCoverageEvent()
        {
            if (_out)
                simout.close(_out);
            delete[] _buffer;
        }

        void process() override;
        void completed(PacketPtr pkt);
        const std::string name() const override;
        const char* description() const override { return "WriteCoverageEvent"; }
    };

  public:

    Tcu(const TcuParams &p);

    ~Tcu();

    void regStats() override;

    System *systemObject() { return system; }

    RegFile &regs() { return regFile; }

    EpFile &eps() { return epFile; }

    TcuConnector &con() { return connector; };

    TcuTlb *tlb() { return tlBuf; }

    bool isMemTile(unsigned tile) const;

    void printLine(Addr len);

    void writeCoverage(PrintReg pr);

    Cycles reset(bool flushInval);

    Cycles flushInvalCaches(bool invalidate);

    void forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest);

    bool isCommandAborting() const
    {
        return cmds.isCommandAborting();
    }

    void scheduleCmdFinish(Cycles delay, TcuError error = TcuError::NONE)
    {
        cmds.scheduleCmdFinish(delay, error);
    }

    void scheduleExtCmdFinish(Cycles delay, TcuError error, RegFile::reg_t arg)
    {
        cmds.scheduleExtCmdFinish(delay, error, arg);
    }

    void sendMemRequest(PacketPtr pkt,
                        Addr data,
                        Cycles delay,
                        bool coverage);

    void sendNocRequest(NocPacketType type,
                        PacketPtr pkt,
                        Cycles delay,
                        bool functional = false);

    void sendNocResponse(PacketPtr pkt, TcuError result = TcuError::NONE);

    NocAddr translatePhysToNoC(Addr phys, bool write);

    void startTransfer(void *event, Cycles delay);

    size_t startForeignReceive(epid_t epId, actid_t actId);

    void printPacket(PacketPtr pkt) const;

  private:

    bool has_message(epid_t ep);

    void completeNocRequest(PacketPtr pkt) override;

    void completeMemRequest(PacketPtr pkt) override;

    void handleNocRequest(PacketPtr pkt) override;

    bool handleCoreMemRequest(PacketPtr pkt,
                              TcuSlavePort &sport,
                              TcuMasterPort &mport,
                              bool icache,
                              bool functional) override;

    bool handleLLCRequest(PacketPtr pkt, bool functional) override;

  private:

    RegFile regFile;

    TcuConnector connector;

    TcuTlb *tlBuf;

    MessageUnit *msgUnit;

    MemoryUnit *memUnit;

    XferUnit *xferUnit;

    CoreRequests coreReqs;

    EpFile epFile;

    TcuCommands cmds;

    EventWrapper<CoreRequests, &CoreRequests::completeReqs> completeCoreReqEvent;

  public:

    const Addr tileMemOffset;

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
    Stats::Scalar resets;

};

#endif // __MEM_TCU_TCU_HH__
