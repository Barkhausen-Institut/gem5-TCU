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

#ifndef __MEM_DTU_XFER_UNIT_HH__
#define __MEM_DTU_XFER_UNIT_HH__

#include "mem/dtu/noc_addr.hh"
#include "mem/dtu/error.hh"
#include "mem/packet.hh"
#include "sim/eventq.hh"
#include "sim/stats.hh"

#include <list>
#include <vector>

class Dtu;

class XferUnit
{
  public:

    enum XferFlags
    {
        NOPF      = 1,
        PRIV      = 2,
        MESSAGE   = 4,
        MSGRECV   = 8,
        NOXLATE   = 16,
    };

    enum AbortType
    {
        ABORT_LOCAL     = 1,
        ABORT_MSGS      = 2,
    };

    class TransferEvent;

    struct Translation
    {
        TransferEvent& event;

        Translation(TransferEvent& _event)
            : event(_event)
        {}

        void abort();

        bool causePagefault();

        void finished(bool success, const NocAddr &phys);
    };

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

    class TransferEvent : public Event
    {
        friend class XferUnit;

      private:

        XferUnit *xfer;

        Buffer *buf;

        uint64_t id;
        Cycles startCycle;
        TransferType type;
        unsigned vpe;
        Addr local;
        size_t remaining;
        uint xferFlags;
        DtuError result;
        Translation *trans;
        int freeSlots;

      public:

        TransferEvent(TransferType _type,
                      Addr _local,
                      size_t _size,
                      uint _flags = 0)
            : xfer(),
              buf(),
              id(nextId++),
              startCycle(),
              type(_type),
              vpe(),
              local(_local),
              remaining(_size),
              xferFlags(_flags),
              result(DtuError::NONE),
              trans(),
              freeSlots()
        {}

        Dtu &dtu() { return xfer->dtu; }

        int bufId() const { return buf->id; }

        uint flags() const { return xferFlags; }

        unsigned vpeId() const { return vpe; }

        void vpeId(unsigned id) { vpe = id; }

        void *data() { return buf->bytes; }

        const void *data() const { return buf->bytes; }

        size_t size() const { return buf->offset; }

        void size(size_t size) { buf->offset = size; }

        const char* description() const override { return "TransferEvent"; }

        const std::string name() const override;

        Translation *translation() { return trans; }

        virtual void transferStart() = 0;

        virtual bool transferDone(DtuError result) = 0;

        void start();

      private:

        void finish()
        {
            assert(trans == NULL);

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

        void translateDone(bool success, const NocAddr &phys);

        void abort(DtuError error);

        static uint64_t nextId;
    };

    XferUnit(Dtu &_dtu, size_t _blockSize, size_t _bufCount, size_t _bufSize);

    ~XferUnit();

    void regStats();

    void startTransfer(TransferEvent *event, Cycles delay);

    bool abortTransfers(uint types);

    void recvMemResponse(uint64_t evId, PacketPtr pkt);

  private:

    void continueTransfer(Buffer *buf);

    Buffer *getBuffer(uint64_t id);

    Buffer* allocateBuf(TransferEvent *event, uint flags);

  private:

    Dtu &dtu;

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
    Stats::Scalar pagefaults;
    Stats::Scalar aborts;
};

#endif
