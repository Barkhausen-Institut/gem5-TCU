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

#ifndef __MEM_DTU_MSG_UNIT_HH__
#define __MEM_DTU_MSG_UNIT_HH__

#include "mem/dtu/dtu.hh"
#include "mem/dtu/mem_unit.hh"

class MessageUnit
{
  public:

    struct MsgInfo
    {
        bool ready;
        bool unlimcred;
        uint8_t flags;
        unsigned targetCoreId;
        unsigned targetEpId;
        unsigned replyEpId;
        uint64_t label;
        uint64_t replyLabel;
        unsigned replyCrd;
    };

    class SendTransferEvent : public MemoryUnit::WriteTransferEvent
    {
        MessageHeader *header;

      public:

        SendTransferEvent(Addr local,
                          size_t size,
                          uint flags,
                          NocAddr dest,
                          MessageHeader *_header)
            : WriteTransferEvent(local, size, flags, dest),
              header(_header)
        {}

        void transferStart() override;
    };

    class ReceiveTransferEvent : public MemoryUnit::ReceiveTransferEvent
    {
        MessageUnit *msgUnit;
        Addr msgAddr;

      public:

        ReceiveTransferEvent(MessageUnit *_msgUnit,
                             Addr local,
                             uint flags,
                             PacketPtr pkt)
            : MemoryUnit::ReceiveTransferEvent(
                Dtu::TransferType::REMOTE_WRITE, local, flags, pkt),
              msgUnit(_msgUnit), msgAddr(local)
        {}

        void transferDone(Dtu::Error result) override;
    };

    MessageUnit(Dtu &_dtu) : dtu(_dtu), info() {}

    void regStats();

    /**
     * Start message transmission -> Mem request
     */
    void startTransmission(const Dtu::Command::Bits& cmd);

    /**
     * Received a message from NoC -> Mem request
     */
    Dtu::Error recvFromNoc(PacketPtr pkt, uint flags);

    /**
     * Finishes the reply-on-message command
     */
    void finishMsgReply(Dtu::Error error, unsigned epid, Addr msgAddr);

    /**
     * Finishes the send-message command
     */
    void finishMsgSend(Dtu::Error error, unsigned epid);

    /**
     * Receives credits again
     */
    void recvCredits(unsigned epid);

    /**
     * Fetches the next message and returns the address or 0
     */
    Addr fetchMessage(unsigned epid);

    /**
     * Acknowledges the message @ <msgAddr>
     */
    Dtu::Error ackMessage(unsigned epId, Addr msgAddr);

    /**
     * Invalidates all reply header for receive EP <repId> that have been sent
     * from PE <peId> and send EP <sepId>.
     */
    Dtu::Error invalidateReply(unsigned repId, unsigned peId, unsigned sepId);

    /**
     * Finishes a message receive
     */
    Dtu::Error finishMsgReceive(unsigned epId,
                                Addr msgAddr,
                                const MessageHeader *header,
                                Dtu::Error error,
                                uint xferFlags);

  private:
    int allocSlot(size_t msgSize, unsigned epid, RecvEp &ep);

    void startXfer(const Dtu::Command::Bits& cmd);

  private:

    Dtu &dtu;

    MsgInfo info;

    Stats::Histogram sentBytes;
    Stats::Histogram repliedBytes;
    Stats::Histogram receivedBytes;
    Stats::Scalar wrongVPE;
    Stats::Scalar noSpace;

};

#endif
