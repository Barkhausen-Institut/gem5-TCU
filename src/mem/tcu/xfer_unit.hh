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

#ifndef __MEM_TCU_XFER_UNIT_HH__
#define __MEM_TCU_XFER_UNIT_HH__

#include "mem/tcu/noc_addr.hh"
#include "mem/tcu/error.hh"
#include "mem/packet.hh"
#include "sim/eventq.hh"
#include "sim/stats.hh"

#include <list>
#include <vector>

class Tcu;

class XferUnit
{
  public:

    enum XferFlags
    {
        MESSAGE   = 1,
        MSGRECV   = 2,
    };

    enum AbortType
    {
        ABORT_LOCAL     = 1,
        ABORT_MSGS      = 2,
    };

    class TransferEvent;

  private:

    struct Buffer
    {
        Buffer(int _id, size_t size)
            : id(_id),
              event(),
              bytes(new uint8_t[size]),
              offset()
        {
        }

        ~Buffer()
        {
            delete[] bytes;
        }

        int id;
        TransferEvent *event;
        uint8_t *bytes;
        size_t offset;
    };

  public:

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

    enum class AbortResult
    {
        NONE,
        WAITING,
        ABORTED,
    };

    class TransferEvent : public Event
    {
        friend class XferUnit;

      private:

        XferUnit *xfer;

        Buffer *buf;

        Cycles startCycle;
        TransferType type;
        NocAddr phys;
        size_t remaining;
        uint xferFlags;
        TcuError result;
        int freeSlots;

      public:

        TransferEvent(TransferType _type,
                      NocAddr _phys,
                      size_t _size,
                      uint _flags = 0)
            : xfer(),
              buf(),
              startCycle(),
              type(_type),
              phys(_phys),
              remaining(_size),
              xferFlags(_flags),
              result(TcuError::NONE),
              freeSlots()
        {}

        Tcu &tcu() { return xfer->tcu; }

        int bufId() const { return buf->id; }

        uint flags() const { return xferFlags; }

        void *data() { return buf->bytes; }

        const void *data() const { return buf->bytes; }

        size_t size() const { return buf->offset; }

        void size(size_t size) { buf->offset = size; }

        const char* description() const override { return "TransferEvent"; }

        const std::string name() const override;

        virtual void transferStart() = 0;

        virtual void transferDone(TcuError result) = 0;

        void start();

      private:

        void finish()
        {
            setFlags(AutoDelete);
        }

        bool isWrite() const
        {
            return type == TransferType::REMOTE_WRITE ||
                   type == TransferType::LOCAL_WRITE;
        }
        bool isRead() const
        {
            return !isWrite();
        }
        bool isRemote() const
        {
            return type == TransferType::REMOTE_READ ||
                   type == TransferType::REMOTE_WRITE;
        }

        void process() override;

        void tryStart();

        AbortResult abort(TcuError error);
    };

    XferUnit(Tcu &_tcu, size_t _blockSize, size_t _bufCount, size_t _bufSize);

    ~XferUnit();

    void regStats();

    void startTransfer(TransferEvent *event, Cycles delay);

    AbortResult tryAbortCommand();

    void recvMemResponse(uint64_t evId, PacketPtr pkt);

  private:

    void continueTransfer(Buffer *buf);

    Buffer* allocateBuf(TransferEvent *event, uint flags);

  private:

    Tcu &tcu;

    size_t blockSize;

    size_t bufCount;
    size_t bufSize;
    Buffer **bufs;

    std::list<TransferEvent*> queue;

    Stats::Histogram reads;
    Stats::Histogram writes;
    Stats::Histogram bytesRead;
    Stats::Histogram bytesWritten;
    Stats::Scalar delays;
    Stats::Scalar aborts;
};

#endif
