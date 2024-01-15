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

#ifndef __MEM_TCU_MSG_UNIT_HH__
#define __MEM_TCU_MSG_UNIT_HH__

#include "mem/tcu/tcu.hh"
#include "mem/tcu/mem_unit.hh"

namespace gem5
{
namespace tcu
{

class MessageUnit
{
  public:

    class SendTransferEvent : public MemoryUnit::WriteTransferEvent
    {
        MessageUnit *msgUnit;
        MessageHeader *header;
        epid_t sepid;

      public:

        SendTransferEvent(MessageUnit *_msgUnit,
                          epid_t _sepid,
                          NocAddr phys,
                          size_t size,
                          unsigned flags,
                          NocAddr dest,
                          MessageHeader *_header)
            : WriteTransferEvent(phys, size, flags, dest),
              msgUnit(_msgUnit),
              header(_header),
              sepid(_sepid)
        {}

        void transferStart() override;
        void transferDone(TcuError result) override;
    };

    class ReceiveTransferEvent : public MemoryUnit::ReceiveTransferEvent
    {
        MessageUnit *msgUnit;
        NocAddr msgAddr;
        EpFile::EpCache *eps;
        epid_t epid;

      public:

        ReceiveTransferEvent(MessageUnit *_msgUnit,
                             EpFile::EpCache *_eps,
                             epid_t _epid,
                             NocAddr phys,
                             unsigned flags,
                             PacketPtr pkt)
            : MemoryUnit::ReceiveTransferEvent(
                XferUnit::TransferType::REMOTE_WRITE, phys, flags, pkt),
              msgUnit(_msgUnit), msgAddr(phys), eps(_eps), epid(_epid)
        {}

        void transferDone(TcuError result) override;
    };

    MessageUnit(Tcu &_tcu)
      : tcu(_tcu),
        sendReplyFinished(true),
        cmdEps(_tcu.eps().newCache(EpFile::Messaging, "msgCmd")),
        extCmdEps(_tcu.eps().newCache(EpFile::Messaging, "msgExtCmd"))
    {}

    void regStats();

    /**
     * Starts the SEND command
     */
    void startSend(const CmdCommand::Bits &cmd);

    /**
     * Starts the REPLY command
     */
    void startReply(const CmdCommand::Bits &cmd);

    /**
     * Starts the FETCH command
     */
    void startFetch(const CmdCommand::Bits &cmd);

    /**
     * Starts the INV_EP command
     */
    void startInvalidate(const ExtCommand::Bits &cmd);

    /**
     * Starts the ACK_MSG command
     */
    void startAck(const CmdCommand::Bits &cmd);

    /**
     * Finishes a SEND or REPLY command with given result.
     *
     * @return true if the SEND/REPLY command is finished now
     */
    bool finishMsgSend(TcuError result);

    /**
     * Received a message from NoC -> Mem request
     */
    void recvFromNoc(PacketPtr pkt);

  private:

    void fetchWithEP(EpFile::EpCache &eps);

    void invalidateWithEP(EpFile::EpCache &eps);

    void startAckWithEP(EpFile::EpCache &eps);

    void ackMessage(RecvEp &rep, int msgidx);

    void recvCredits(EpFile::EpCache &eps, SendEp &sep);

    int allocSlot(EpFile::EpCache &eps, RecvEp &ep, Cycles *delay);

    void startReplyWithEP(EpFile::EpCache &eps);

    void startSendReplyWithEP(EpFile::EpCache &eps, epid_t epid);

    void recvFromNocWithEP(EpFile::EpCache &eps, PacketPtr pkt);

    void finishMsgSendWithEp(EpFile::EpCache &eps, TcuError result);

    TcuError finishMsgReceive(EpFile::EpCache &eps,
                              RecvEp &ep,
                              Addr msgAddr,
                              const MessageHeader *header,
                              TcuError error,
                              unsigned xferFlags,
                              bool addMsg);

  private:

    Tcu &tcu;

    bool sendReplyFinished;
    EpFile::EpCache cmdEps;
    EpFile::EpCache extCmdEps;

    statistics::Histogram sentBytes;
    statistics::Histogram repliedBytes;
    statistics::Histogram receivedBytes;
    statistics::Scalar wrongAct;
    statistics::Scalar noSpace;

};

}
}

#endif
