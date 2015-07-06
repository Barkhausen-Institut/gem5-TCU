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

#ifndef __MEM_DTU_DTU_HH__
#define __MEM_DTU_DTU_HH__

#include "mem/dtu/base.hh"
#include "mem/dtu/regfile.hh"
#include "params/Dtu.hh"

class Dtu : public BaseDtu
{
  private:

    enum MessageFlags : uint8_t
    {
        REPLY_FLAG = (1 << 0),
        GRAND_CREDITS_FLAG = (1 << 1),
    };

    struct MessageHeader
    {
        uint8_t flags; // if bit 0 is set its a reply, if bit 1 is set we grand credits
        uint8_t senderCoreId;
        uint8_t senderEpId;
        uint8_t replyEpId; // for a normal message this is the reply epId
                           // for a reply this is the enpoint that receives credits
        uint16_t length;
    };

    enum class NocPacketType
    {
        MESSAGE,
        REQUEST,
    };

    enum class SpmPacketType
    {
        FORWARDED_MESSAGE,
        FORWARDED_REQUEST,
        LOCAL_REQUEST,
    };

    struct SpmSenderState : public Packet::SenderState
    {
        SpmPacketType packetType;
        unsigned epId; // only valid if packetType != SpmPacketType::FORWARDER_REUEST
    };

    struct NocSenderState : public Packet::SenderState
    {
        NocPacketType packetType;
    };

    enum class CommandOpcode
    {
        IDLE = 0,
        START_OPERATION = 1,
        INC_READ_PTR = 2,
    };

    static constexpr unsigned numCmdOpcodeBits = 2;

    struct Command
    {
        CommandOpcode opcode;
        unsigned epId;
        Addr offset;
    };

    enum class EpMode
    {
        RECEIVE_MESSAGE,
        TRANSMIT_MESSAGE,
        READ_MEMORY,
        WRITE_MEMORY,
    };

    bool atomicMode;

    System *system;

    RegFile regFile;

    unsigned numEndpoints;

    MasterID masterId;

    Addr maxNocPacketSize;

    unsigned numCmdEpidBits;
    unsigned numCmdOffsetBits;

    Cycles registerAccessLatency;
    Cycles commandToSpmRequestLatency;
    Cycles commandToNocRequestLatency;
    Cycles spmResponseToNocRequestLatency;
    Cycles nocMessageToSpmRequestLatency;
    Cycles nocResponseToSpmRequestLatency;
    Cycles nocRequestToSpmRequestLatency;
    Cycles spmResponseToNocResponseLatency;

    bool usePTable;

    PacketPtr generateRequest(Addr addr, Addr size, MemCmd cmd);

    Command getCommand();

    void executeCommand();
    EventWrapper<Dtu, &Dtu::executeCommand> executeCommandEvent;

    void startOperation(Command& cmd);

    void finishOperation();
    EventWrapper<Dtu, &Dtu::finishOperation> finishOperationEvent;

    void sendSpmRequest(PacketPtr pkt,
                        unsigned epId,
                        Cycles delay,
                        SpmPacketType packetType);

    void startMessageTransmission(const Command& cmd);

    void startMemoryRead(const Command& cmd);

    void startMemoryWrite(const Command& cmd);

    void incrementReadPtr(unsigned epId);

    void incrementWritePtr(unsigned epId);

    struct IncrementWritePtrEvent : public Event
    {
        unsigned epId = 0;

        Dtu& dtu;

        IncrementWritePtrEvent(Dtu& _dtu) : dtu(_dtu) {}

        void process() override { dtu.incrementWritePtr(epId); }

        const char* description() const override { return "IncrementWritePtrEvent"; }

        const std::string name() const override { return dtu.name(); }
    };

    IncrementWritePtrEvent incrementWritePtrEvent;

    void forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest);

    void sendNocRequest(PacketPtr pkt,
                        Cycles delay,
                        bool isMessage);

    void sendNocMessage(const uint8_t* data,
                        Addr messageSize,
                        Tick spmPktHeaderDelay,
                        Tick spmPktPayloadDelay);

    void sendNocMemoryWriteRequest(const uint8_t* data,
                                   Addr requestSize,
                                   Tick spmPktHeaderDelay,
                                   Tick spmPktPayloadDelay);

    void completeLocalSpmRequest(PacketPtr pkt);

    void completeForwardedMessage(PacketPtr pkt, unsigned epId);

    void completeForwardedRequest(PacketPtr pkt);

    void completeNocRequest(PacketPtr pkt) override;

    void completeNocReadRequest(PacketPtr pkt);

    void completeNocWriteRequest(PacketPtr pkt);

    void completeSpmRequest(PacketPtr pkt) override;

    void handleNocRequest(PacketPtr pkt) override;

    void handleCpuRequest(PacketPtr pkt) override;

    void recvNocMessage(PacketPtr pkt);

    void recvNocMemoryRequest(PacketPtr pkt);

  public:

    Dtu(DtuParams* p);
};

#endif // __MEM_DTU_DTU_HH__
