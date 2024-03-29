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

#ifndef __MEM_TCU_MEM_UNIT_HH__
#define __MEM_TCU_MEM_UNIT_HH__

#include "mem/tcu/tcu.hh"
#include "mem/tcu/ep_file.hh"
#include "mem/tcu/xfer_unit.hh"

namespace gem5
{
namespace tcu
{

class MemoryUnit
{
  public:

    class LocalWriteTransferEvent : public XferUnit::TransferEvent
    {
        uint8_t *tmp;
        size_t tmpSize;

      public:

        LocalWriteTransferEvent(NocAddr phys, uint8_t *_tmp,
                                size_t _size, unsigned flags)
            : TransferEvent(XferUnit::TransferType::LOCAL_WRITE,
                            phys, _size, flags),
              tmp(_tmp),
              tmpSize(_size)
        {}

        void transferStart() override;

        void transferDone(TcuError result) override;
    };

    class ReadTransferEvent : public XferUnit::TransferEvent
    {
        PacketPtr pkt;

      public:

        ReadTransferEvent(NocAddr phys, unsigned flags, PacketPtr _pkt)
            : TransferEvent(XferUnit::TransferType::LOCAL_WRITE,
                            phys, _pkt->getSize(), flags),
              pkt(_pkt)
        {}

        void transferStart() override;

        void transferDone(TcuError result) override;
    };

    class WriteTransferEvent : public XferUnit::TransferEvent
    {
      protected:

        NocAddr dest;

      public:

        WriteTransferEvent(NocAddr phys,
                           size_t size,
                           unsigned flags,
                           NocAddr _dest)
            : TransferEvent(XferUnit::TransferType::LOCAL_READ,
                            phys, size, flags),
              dest(_dest)
        {}

        void transferStart() override {}

        void transferDone(TcuError result) override;
    };

    class ReceiveTransferEvent : public XferUnit::TransferEvent
    {
      protected:

        PacketPtr pkt;

      public:

        ReceiveTransferEvent(XferUnit::TransferType type,
                             NocAddr phys,
                             unsigned flags,
                             PacketPtr _pkt)
            : XferUnit::TransferEvent(type, phys, _pkt->getSize(), flags),
              pkt(_pkt)
        {}

        void transferStart() override;

        void transferDone(TcuError result) override;
    };

    MemoryUnit(Tcu &_tcu) : tcu(_tcu), eps(_tcu.eps().newCache()) {}

    void regStats();

    /**
     * Starts a read -> NoC request
     */
    void startRead(const CmdCommand::Bits& cmd);

    /**
     * Starts a write -> Mem request
     */
    void startWrite(const CmdCommand::Bits& cmd);

    /**
     * Read: response from remote TCU
     */
    void readComplete(const CmdCommand::Bits& cmd, PacketPtr pkt,
                      TcuError error);

    /**
     * Write: response from remote TCU
     */
    void writeComplete(const CmdCommand::Bits& cmd, PacketPtr pkt,
                       TcuError error);


    /**
     * Functional access from NoC
     */
    void recvFunctionalFromNoc(PacketPtr pkt);

    /**
     * Received read/write request from NoC -> Mem/regfile request
     */
    void recvFromNoc(PacketPtr pkt);

  private:

    void startReadWithEP(EpFile::EpCache &eps);

    void startWriteWithEP(EpFile::EpCache &eps);

    Tcu &tcu;

    EpFile::EpCache eps;

    statistics::Histogram readBytes;
    statistics::Histogram writtenBytes;
    statistics::Histogram receivedBytes;
    statistics::Scalar wrongAct;

};

}
}

#endif
