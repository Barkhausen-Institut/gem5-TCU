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

#ifndef __MEM_DTU_MEM_UNIT_HH__
#define __MEM_DTU_MEM_UNIT_HH__

#include "mem/dtu/dtu.hh"
#include "mem/dtu/xfer_unit.hh"

class MemoryUnit
{
  public:

    class LocalReadTransferEvent : public XferUnit::TransferEvent
    {
        Addr dest;

      public:

        LocalReadTransferEvent(Addr src, Addr _dest, size_t size, uint flags)
            : TransferEvent(Dtu::TransferType::LOCAL_READ,
                            src,
                            size,
                            flags),
              dest(_dest)
        {}

        void transferStart() override {}

        void transferDone(Dtu::Error result) override;
    };

    class LocalWriteTransferEvent : public XferUnit::TransferEvent
    {
        uint8_t *tmp;
        size_t tmpSize;

      public:

        LocalWriteTransferEvent(Addr local, uint8_t *_tmp, size_t _size, uint flags)
            : TransferEvent(Dtu::TransferType::LOCAL_WRITE,
                            local,
                            _size,
                            flags),
              tmp(_tmp),
              tmpSize(_size)
        {}

        void transferStart() override;

        void transferDone(Dtu::Error result) override;
    };

    class ReadTransferEvent : public XferUnit::TransferEvent
    {
        PacketPtr pkt;

      public:

        ReadTransferEvent(Addr local, uint flags, PacketPtr _pkt)
            : TransferEvent(Dtu::TransferType::LOCAL_WRITE,
                            local,
                            _pkt->getSize(),
                            flags),
              pkt(_pkt)
        {}

        void transferStart() override;

        void transferDone(Dtu::Error result) override;
    };

    class WriteTransferEvent : public XferUnit::TransferEvent
    {
      protected:

        NocAddr dest;
        uint vpeId;

      public:

        WriteTransferEvent(Addr local,
                           size_t size,
                           uint flags,
                           NocAddr _dest,
                           uint _vpeId)
            : TransferEvent(Dtu::TransferType::LOCAL_READ, local, size, flags),
              dest(_dest),
              vpeId(_vpeId)
        {}

        void transferStart() override {};

        void transferDone(Dtu::Error result) override;
    };

    class ReceiveTransferEvent : public XferUnit::TransferEvent
    {
      protected:

        PacketPtr pkt;

      public:

        ReceiveTransferEvent(Dtu::TransferType type,
                             Addr local,
                             uint flags,
                             PacketPtr _pkt)
            : XferUnit::TransferEvent(type, local, _pkt->getSize(), flags),
              pkt(_pkt)
        {}

        void transferStart() override;

        void transferDone(Dtu::Error result) override;
    };

    MemoryUnit(Dtu &_dtu) : dtu(_dtu) {}

    void regStats();

    /**
     * Starts a read -> NoC request
     */
    void startRead(const Dtu::Command& cmd);

    /**
     * Starts a write -> Mem request
     */
    void startWrite(const Dtu::Command& cmd);

    /**
     * Read: response from remote DTU
     */
    void readComplete(const Dtu::Command& cmd, PacketPtr pkt, Dtu::Error error);

    /**
     * Write: response from remote DTU
     */
    void writeComplete(const Dtu::Command& cmd, PacketPtr pkt, Dtu::Error error);


    /**
     * Functional access from NoC
     */
    void recvFunctionalFromNoc(PacketPtr pkt);

    /**
     * Received read/write request from NoC -> Mem/regfile request
     */
    Dtu::Error recvFromNoc(PacketPtr pkt, uint vpeId, uint flags);

  private:

    Dtu &dtu;

    Stats::Histogram readBytes;
    Stats::Histogram writtenBytes;
    Stats::Histogram receivedBytes;
    Stats::Scalar wrongVPE;

};

#endif
