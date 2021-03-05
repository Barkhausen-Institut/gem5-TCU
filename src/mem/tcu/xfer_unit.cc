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

#include "debug/Tcu.hh"
#include "debug/TcuBuf.hh"
#include "debug/TcuPackets.hh"
#include "debug/TcuXfers.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/xfer_unit.hh"

static const char *decodeFlags(uint flags)
{
    static char buf[3];
    buf[0] = (flags & XferUnit::MESSAGE) ? 'm' : '-';
    buf[1] = (flags & XferUnit::MSGRECV) ? 'r' : '-';
    buf[2] = '\0';
    return buf;
}

XferUnit::XferUnit(Tcu &_tcu,
                   size_t _blockSize,
                   size_t _bufCount,
                   size_t _bufSize)
    : tcu(_tcu),
      blockSize(_blockSize),
      bufCount(_bufCount),
      bufSize(_bufSize),
      bufs(new Buffer*[bufCount]),
      queue()
{
    for (size_t i = 0; i < bufCount; ++i)
        bufs[i] = new Buffer(i, bufSize);
}

XferUnit::~XferUnit()
{
    for (size_t i = 0; i < bufCount; ++i)
        delete bufs[i];
    delete[] bufs;
}

void
XferUnit::regStats()
{
    reads
        .init(8)
        .name(tcu.name() + ".xfer.reads")
        .desc("Read times (in Cycles)")
        .flags(Stats::nozero);
    writes
        .init(8)
        .name(tcu.name() + ".xfer.writes")
        .desc("Write times (in Cycles)")
        .flags(Stats::nozero);
    bytesRead
        .init(8)
        .name(tcu.name() + ".xfer.bytesRead")
        .desc("Read bytes (from internal memory)")
        .flags(Stats::nozero);
    bytesWritten
        .init(8)
        .name(tcu.name() + ".xfer.bytesWritten")
        .desc("Written bytes (to internal memory)")
        .flags(Stats::nozero);
    delays
        .name(tcu.name() + ".xfer.delays")
        .desc("Number of delays due to occupied buffers");
    aborts
        .name(tcu.name() + ".xfer.aborts")
        .desc("Number of aborts");
}

void
XferUnit::TransferEvent::tryStart()
{
    assert(buf == NULL);

    buf = xfer->allocateBuf(this, flags());

    // try again later, if there is no free buffer
    if (!buf)
    {
        DPRINTFS(TcuXfers, (&xfer->tcu),
            "Delaying %s transfer of %lu bytes @ %p [flags=%s]\n",
            isWrite() ? "mem-write" : "mem-read",
            remaining,
            phys.getAddr(),
            decodeFlags(flags()));

        xfer->delays++;
        xfer->queue.push_back(this);
        return;
    }

    transferStart();
    start();
}

void
XferUnit::TransferEvent::start()
{
    // workaround for race condition: if the abort request was issued right
    // after we've created the transfer, but before we've allocated a buffer,
    // we missed the abort. so, check for the abort here again.
    if (!isRemote() && xfer->tcu.isCommandAborting())
    {
        abort(TcuError::ABORT);
        return;
    }

    DPRINTFS(TcuXfers, (&xfer->tcu),
        "buf%d: Starting %s transfer of %lu bytes @ %p [flags=%s]\n",
        buf->id,
        isWrite() ? "mem-write" : "mem-read",
        remaining,
        phys.getAddr(),
        decodeFlags(flags()));

    xfer->tcu.schedule(this, xfer->tcu.clockEdge(Cycles(1)));
}

const std::string
XferUnit::TransferEvent::name() const
{
    return xfer->tcu.name();
}

void
XferUnit::TransferEvent::process()
{
    if (!buf)
    {
        tryStart();
        return;
    }

    if (remaining == 0)
    {
        xfer->continueTransfer(buf);
        return;
    }

    // if there was an error, we have aborted it on purpose
    // in this case, TransferEvent::abort() will do the rest
    if (result != TcuError::NONE)
        return;

    Addr physAddr = phys.getAddr();

    while(freeSlots > 0 && remaining > 0)
    {
        Addr physOff = physAddr & (xfer->blockSize - 1);
        Addr reqSize = std::min(remaining, xfer->blockSize - physOff);

        auto cmd = isWrite() ? MemCmd::WriteReq : MemCmd::ReadReq;
        auto pkt = xfer->tcu.generateRequest(physAddr, reqSize, cmd);

        DPRINTFS(TcuXfers, (&xfer->tcu),
            "buf%d: %s %lu bytes @ %p in local memory\n",
            buf->id,
            isWrite() ? "Writing" : "Reading",
            reqSize,
            physAddr);

        Cycles lat = xfer->tcu.transferToMemRequestLatency;

        if (isWrite())
        {
            assert(buf->offset + reqSize <= xfer->bufSize);

            memcpy(pkt->getPtr<uint8_t>(), buf->bytes + buf->offset, reqSize);
        }

        xfer->tcu.sendMemRequest(pkt, buf->id | (buf->offset << 32), lat);

        // to next block
        buf->offset += reqSize;
        physAddr += reqSize;
        remaining -= reqSize;
        freeSlots--;
    }

    // remember our position
    phys = NocAddr(physAddr);
}

void
XferUnit::recvMemResponse(uint64_t id_off, PacketPtr pkt)
{
    Addr offset = id_off >> 32;
    Buffer *buf = bufs[id_off & 0xFFFFFFFF];
    assert(buf != nullptr);

    if (pkt)
    {
        if (buf->event->isRead())
        {
            assert(offset + pkt->getSize() <= bufSize);

            memcpy(buf->bytes + offset,
                   pkt->getConstPtr<uint8_t>(),
                   pkt->getSize());
        }

        buf->event->freeSlots++;
    }

    DPRINTFS(TcuXfers, (&tcu),
             "buf%d: Received mem response for %#lx (rem=%#lx, slots=%d/%d)\n",
             buf->id, offset, buf->event->remaining,
             buf->event->freeSlots, tcu.reqCount);

    continueTransfer(buf);
}

void
XferUnit::continueTransfer(Buffer *buf)
{
    // transfer done or aborted, but all memory responses received?
    if (buf->event->freeSlots == tcu.reqCount &&
        (buf->event->result != TcuError::NONE || buf->event->remaining == 0))
    {
        buf->event->transferDone(buf->event->result);

        DPRINTFS(TcuXfers, (&tcu), "buf%d: Transfer done\n", buf->id);

        // we're done with this buffer now
        if (buf->event->isRead())
            reads.sample(tcu.curCycle() - buf->event->startCycle);
        else
            writes.sample(tcu.curCycle() - buf->event->startCycle);
        buf->event->finish();
        buf->event = NULL;

        // start the next one, if there is any
        if (!queue.empty())
        {
            TransferEvent *ev = queue.front();
            queue.pop_front();
            tcu.schedule(ev, tcu.clockEdge(Cycles(1)));
        }
    }
    // continue if there was no error and there is something left to transfer
    else if(buf->event->result == TcuError::NONE && buf->event->remaining > 0)
        buf->event->process();
}

XferUnit::AbortResult
XferUnit::TransferEvent::abort(TcuError error)
{
    // if there are pending memory responses, mark it as aborted, but continue
    if (buf->event->freeSlots != xfer->tcu.reqCount)
    {
        buf->event->result = error;
        return AbortResult::WAITING;
    }

    DPRINTFS(TcuXfers, (&xfer->tcu),
        "buf%d: aborting transfer (%d)\n",
        buf->id,
        static_cast<int>(error));

    buf->event->result = error;

    xfer->aborts++;

    if(scheduled())
        xfer->tcu.deschedule(this);

    buf->event->remaining = 0;
    xfer->recvMemResponse(buf->id, NULL);
    return AbortResult::ABORTED;
}

void
XferUnit::startTransfer(TransferEvent *event, Cycles delay)
{
    event->xfer = this;
    event->freeSlots = tcu.reqCount;
    event->startCycle = tcu.curCycle();

    if (event->isRead())
        bytesRead.sample(event->remaining);
    else
        bytesWritten.sample(event->remaining);

    tcu.schedule(event, tcu.clockEdge(Cycles(delay + 1)));

    // finish the noc request now to make the port unbusy
    if (event->isRemote())
        tcu.schedNocRequestFinished(tcu.clockEdge(Cycles(1)));
}

XferUnit::AbortResult
XferUnit::tryAbortCommand()
{
    for (size_t i = 0; i < bufCount; ++i)
    {
        auto ev = bufs[i]->event;
        if (ev && !ev->isRemote())
            return ev->abort(TcuError::ABORT);
    }
    return AbortResult::NONE;
}

XferUnit::Buffer*
XferUnit::allocateBuf(TransferEvent *event, uint flags)
{
    for (size_t i = 0; i < bufCount; ++i)
    {
        if (!bufs[i]->event)
        {
            bufs[i]->event = event;
            bufs[i]->offset = 0;
            return bufs[i];
        }
    }

    return NULL;
}
