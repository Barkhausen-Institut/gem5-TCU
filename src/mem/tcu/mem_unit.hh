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

#ifndef __MEM_TCU_MEM_UNIT_HH__
#define __MEM_TCU_MEM_UNIT_HH__

#include "mem/tcu/tcu.hh"
#include "mem/tcu/xfer_unit.hh"

class MemoryUnit
{
  public:

    class LocalReadTransferEvent : public XferUnit::TransferEvent
    {
        Addr dest;

      public:

        LocalReadTransferEvent(Addr src, Addr _dest, size_t size, uint flags)
            : TransferEvent(XferUnit::TransferType::LOCAL_READ,
                            src,
                            size,
                            flags),
              dest(_dest)
        {}

        void transferStart() override {}

        bool transferDone(TcuError result) override;
    };

    class LocalWriteTransferEvent : public XferUnit::TransferEvent
    {
        uint8_t *tmp;
        size_t tmpSize;

      public:

        LocalWriteTransferEvent(Addr local, uint8_t *_tmp, size_t _size, uint flags)
            : TransferEvent(XferUnit::TransferType::LOCAL_WRITE,
                            local,
                            _size,
                            flags),
              tmp(_tmp),
              tmpSize(_size)
        {}

        void transferStart() override;

        bool transferDone(TcuError result) override;
    };

    class ReadTransferEvent : public XferUnit::TransferEvent
    {
        PacketPtr pkt;

      public:

        ReadTransferEvent(Addr local, uint flags, PacketPtr _pkt)
            : TransferEvent(XferUnit::TransferType::LOCAL_WRITE,
                            local,
                            _pkt->getSize(),
                            flags),
              pkt(_pkt)
        {}

        void transferStart() override;

        bool transferDone(TcuError result) override;
    };

    class WriteTransferEvent : public XferUnit::TransferEvent
    {
      protected:

        NocAddr dest;
        vpeid_t vpe;

      public:

        WriteTransferEvent(Addr local,
                           size_t size,
                           vpeid_t _vpe,
                           uint flags,
                           NocAddr _dest)
            : TransferEvent(XferUnit::TransferType::LOCAL_READ, local, size, flags),
              dest(_dest), vpe(_vpe)
        {}

        void transferStart() override {}

        bool transferDone(TcuError result) override;
    };

    class ReceiveTransferEvent : public XferUnit::TransferEvent
    {
      protected:

        PacketPtr pkt;
        vpeid_t vpe;

      public:

        ReceiveTransferEvent(XferUnit::TransferType type,
                             Addr local,
                             vpeid_t _vpe,
                             uint flags,
                             PacketPtr _pkt)
            : XferUnit::TransferEvent(type, local, _pkt->getSize(), flags),
              pkt(_pkt), vpe(_vpe)
        {}

        void transferStart() override;

        bool transferDone(TcuError result) override;
    };

    MemoryUnit(Tcu &_tcu) : tcu(_tcu) {}

    void regStats();

    /**
     * Starts a read -> NoC request
     */
    void startRead(const Tcu::Command::Bits& cmd);

    /**
     * Starts a write -> Mem request
     */
    void startWrite(const Tcu::Command::Bits& cmd);

    /**
     * Read: response from remote TCU
     */
    void readComplete(const Tcu::Command::Bits& cmd, PacketPtr pkt, TcuError error);

    /**
     * Write: response from remote TCU
     */
    void writeComplete(const Tcu::Command::Bits& cmd, PacketPtr pkt, TcuError error);


    /**
     * Functional access from NoC
     */
    void recvFunctionalFromNoc(PacketPtr pkt);

    /**
     * Received read/write request from NoC -> Mem/regfile request
     */
    TcuError recvFromNoc(vpeid_t tvpe, PacketPtr pkt, uint flags);

  private:

    Tcu &tcu;

    Stats::Histogram readBytes;
    Stats::Histogram writtenBytes;
    Stats::Histogram receivedBytes;
    Stats::Scalar wrongVPE;

};

#endif
