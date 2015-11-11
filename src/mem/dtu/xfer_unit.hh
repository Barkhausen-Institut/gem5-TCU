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

#include "mem/dtu/dtu.hh"
#include "mem/dtu/noc_addr.hh"

class XferUnit
{
  public:

    enum XferFlags
    {
        MESSAGE   = 1,
        LAST      = 2,
        CACHEMISS = 4,
    };

  private:

    struct Buffer;

    struct TransferEvent : public Event
    {
        XferUnit& xfer;

        Buffer *buf;

        Dtu::TransferType type;
        Addr localAddr;
        NocAddr remoteAddr;
        size_t size;
        PacketPtr pkt;
        uint flags;

        TransferEvent(XferUnit& _xfer)
            : xfer(_xfer),
              buf(),
              type(),
              localAddr(),
              remoteAddr(),
              size(),
              pkt(),
              flags()
        {}

        void process() override;

        void translateDone(bool success, const NocAddr &phys);

        const char* description() const override { return "TransferEvent"; }

        const std::string name() const override { return xfer.dtu.name(); }
    };

    struct Translation : PtUnit::Translation
    {
        TransferEvent& event;

        Translation(TransferEvent& _event)
            : event(_event)
        {}

        void finished(bool success, const NocAddr &phys) override
        {
            event.translateDone(success, phys);

            delete this;
        }
    };

    struct Buffer
    {
        Buffer(XferUnit& _xfer, int _id, size_t size)
            : id(_id),
              event(_xfer),
              bytes(new uint8_t[size]),
              offset(),
              free(true)
        {
            event.buf = this;
        }

        ~Buffer()
        {
            delete[] bytes;
        }

        int id;
        TransferEvent event;
        uint8_t *bytes;
        size_t offset;
        bool free;
    };

    struct StartEvent : public Event
    {
        XferUnit& xfer;

        Dtu::TransferType type;
        NocAddr remoteAddr;
        Addr localAddr;
        size_t size;
        PacketPtr pkt;
        Dtu::MessageHeader* header;
        uint flags;

        StartEvent(XferUnit& _xfer,
                   Dtu::TransferType _type,
                   NocAddr _remoteAddr,
                   Addr _localAddr,
                   size_t _size,
                   PacketPtr _pkt,
                   Dtu::MessageHeader* _header,
                   uint _flags)
            : xfer(_xfer),
              type(_type),
              remoteAddr(_remoteAddr),
              localAddr(_localAddr),
              size(_size),
              pkt(_pkt),
              header(_header),
              flags(_flags)
        {}

        void process() override
        {
            // the delay was already paid earlier
            if(xfer.startTransfer(type,
                                  remoteAddr,
                                  localAddr,
                                  size,
                                  pkt,
                                  header,
                                  Cycles(0),
                                  flags))
            {
                setFlags(AutoDelete);
            }
        }

        const char* description() const override { return "StartXferEvent"; }

        const std::string name() const override { return xfer.dtu.name(); }
    };

  public:

    XferUnit(Dtu &_dtu, size_t _blockSize, size_t _bufCount, size_t _bufSize);

    ~XferUnit();

    bool startTransfer(Dtu::TransferType type,
                       NocAddr remoteAddr,
                       Addr localAddr,
                       size_t size,
                       PacketPtr pkt,
                       Dtu::MessageHeader* header,
                       Cycles delay,
                       uint flags);

    void recvMemResponse(size_t bufId,
                         const void* data,
                         size_t size,
                         Tick headerDelay,
                         Tick payloadDelay);

  private:

    Buffer* allocateBuf(bool cacheMiss);

  private:

    Dtu &dtu;

    size_t blockSize;

    size_t bufCount;
    size_t bufSize;
    Buffer **bufs;
};

#endif
