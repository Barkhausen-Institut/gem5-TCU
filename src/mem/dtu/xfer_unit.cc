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

#include "debug/Dtu.hh"
#include "debug/DtuBuf.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuXfers.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/xfer_unit.hh"

uint64_t XferUnit::TransferEvent::nextId = 0;

static const char *decodeFlags(uint flags)
{
    static char buf[5];
    buf[0] = (flags & XferUnit::MESSAGE) ? 'm' : '-';
    buf[1] = (flags & XferUnit::MSGRECV) ? 'r' : '-';
    buf[2] = (flags & XferUnit::NOPF)    ? 'p' : '-';
    buf[3] = (flags & XferUnit::NOXLATE) ? 'x' : '-';
    buf[4] = '\0';
    return buf;
}

XferUnit::XferUnit(Dtu &_dtu,
                   size_t _blockSize,
                   size_t _bufCount,
                   size_t _bufSize)
    : dtu(_dtu),
      blockSize(_blockSize),
      bufCount(_bufCount),
      bufSize(_bufSize),
      bufs(new Buffer*[bufCount]),
      queue()
{
    panic_if(dtu.tlb() && bufCount < 2,
        "With paging enabled, at least 2 buffers are required");

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
        .name(dtu.name() + ".xfer.reads")
        .desc("Read times (in Cycles)")
        .flags(Stats::nozero);
    writes
        .init(8)
        .name(dtu.name() + ".xfer.writes")
        .desc("Write times (in Cycles)")
        .flags(Stats::nozero);
    bytesRead
        .init(8)
        .name(dtu.name() + ".xfer.bytesRead")
        .desc("Read bytes (from internal memory)")
        .flags(Stats::nozero);
    bytesWritten
        .init(8)
        .name(dtu.name() + ".xfer.bytesWritten")
        .desc("Written bytes (to internal memory)")
        .flags(Stats::nozero);
    delays
        .name(dtu.name() + ".xfer.delays")
        .desc("Number of delays due to occupied buffers");
    pagefaults
        .name(dtu.name() + ".xfer.pagefaults")
        .desc("Number of pagefaults during transfers");
    aborts
        .name(dtu.name() + ".xfer.aborts")
        .desc("Number of aborts");
}

void
XferUnit::Translation::abort()
{
    event.xfer->dtu.abortTranslate(event.buf->id);
}

bool
XferUnit::Translation::causePagefault()
{
    return !(event.flags() & XferFlags::NOPF);
}

void
XferUnit::Translation::finished(bool success, const NocAddr &phys)
{
    event.translateDone(success, phys);

    delete this;
}

void
XferUnit::TransferEvent::tryStart()
{
    assert(buf == NULL);

    buf = xfer->allocateBuf(this, flags());

    // try again later, if there is no free buffer
    if (!buf)
    {
        DPRINTFS(DtuXfers, (&xfer->dtu),
            "Delaying %s transfer of %lu bytes @ %p [flags=%s]\n",
            isWrite() ? "mem-write" : "mem-read",
            remaining,
            local,
            decodeFlags(flags()));

        xfer->delays++;
        xfer->queue.push_back(this);
        return;
    }

    if (transferStart())
        start();
}

void
XferUnit::TransferEvent::start()
{
    DPRINTFS(DtuXfers, (&xfer->dtu),
        "buf%d: Starting %s transfer of %lu bytes @ %p [flags=%s]\n",
        buf->id,
        isWrite() ? "mem-write" : "mem-read",
        remaining,
        local,
        decodeFlags(flags()));

    xfer->dtu.schedule(this, xfer->dtu.clockEdge(Cycles(1)));
}

const std::string
XferUnit::TransferEvent::name() const
{
    return xfer->dtu.name();
}

void
XferUnit::TransferEvent::process()
{
    if (!buf)
    {
        tryStart();
        return;
    }

    NocAddr phys(local);
    if (xfer->dtu.tlb() && !(flags() & NOXLATE))
    {
        // the DTU can always write to receive buffers and they are pinned
        // so, don't check for write access in that case
        uint access = isWrite() && !(flags() & MSGRECV) ? DtuTlb::WRITE
                                                        : DtuTlb::READ;
        access |= isRemote() ? 0 : DtuTlb::INTERN;

        DtuTlb::Result res = xfer->dtu.tlb()->lookup(local, access, &phys);
        if (res != DtuTlb::HIT)
        {
            if (res == DtuTlb::PAGEFAULT)
                xfer->pagefaults++;

            // if this is a pagefault and we are not allowed to cause one,
            // report an error
            if (res == DtuTlb::PAGEFAULT && (flags() & XferFlags::NOPF))
            {
                abort(DtuError::PAGEFAULT);
                return;
            }

            assert(res != DtuTlb::NOMAP);
            trans = new Translation(*this);
            xfer->dtu.startTranslate(buf->id, local, access, trans);
            return;
        }
    }

    translateDone(true, phys);
}

void
XferUnit::TransferEvent::translateDone(bool success, const NocAddr &phys)
{
    // if there was an error, we have aborted it on purpose
    // in this case, TransferEvent::abort() will do the rest
    if (result != DtuError::NONE)
        return;

    trans = NULL;

    if (!success)
    {
        abort(DtuError::PAGEFAULT);
        return;
    }

    // it might happen that we called process() multiple times in a row. if
    // another one already generated all remaining memory requests, we are
    // done here
    if (remaining == 0)
        return;

    Addr nextPage = local + DtuTlb::PAGE_SIZE;
    nextPage &= ~static_cast<Addr>(DtuTlb::PAGE_MASK);
    Addr pageRemaining = std::min(remaining, nextPage - local);
    // local might have been moved forward
    // make sure to use the correct page offset
    Addr physAddr = phys.getAddr() & ~static_cast<Addr>(DtuTlb::PAGE_MASK);
    physAddr = xfer->dtu.nocToPhys(physAddr);
    physAddr += local & DtuTlb::PAGE_MASK;

    while(freeSlots > 0 && pageRemaining > 0)
    {
        Addr localOff = local & (xfer->blockSize - 1);
        Addr reqSize = std::min(remaining, xfer->blockSize - localOff);

        auto cmd = isWrite() ? MemCmd::WriteReq : MemCmd::ReadReq;
        auto pkt = xfer->dtu.generateRequest(physAddr, reqSize, cmd);

        DPRINTFS(DtuXfers, (&xfer->dtu),
            "buf%d: %s %lu bytes @ %p->%p in local memory\n",
            buf->id,
            isWrite() ? "Writing" : "Reading",
            reqSize,
            local,
            physAddr);

        Cycles lat = xfer->dtu.transferToMemRequestLatency;

        if (isWrite())
        {
            assert(buf->offset + reqSize <= xfer->bufSize);

            memcpy(pkt->getPtr<uint8_t>(), buf->bytes + buf->offset, reqSize);
        }

        xfer->dtu.sendMemRequest(pkt, local, id | (buf->offset << 32), lat);

        // to next block
        local += reqSize;
        buf->offset += reqSize;
        physAddr += reqSize;
        remaining -= reqSize;
        pageRemaining -= reqSize;
        freeSlots--;
    }
}

void
XferUnit::recvMemResponse(uint64_t evId, PacketPtr pkt)
{
    Buffer *buf = getBuffer(evId & 0xFFFFFFFF);
    // ignore responses for aborted transfers
    if (!buf || !buf->event)
    {
        DPRINTFS(DtuXfers, (&dtu),
                 "buf%d: Ignoring mem response (buf=%#lx, event=%#lx)\n",
                 (evId & 0xFFFFFFFF), buf, buf ? buf->event : nullptr);
        return;
    }

    DPRINTFS(DtuXfers, (&dtu),
             "buf%d: Received mem response for %#lx (rem=%#lx, slots=%d/%d)\n",
             buf->id, (Addr)(evId >> 32), buf->event->remaining,
             buf->event->freeSlots + 1, dtu.reqCount);

    if (pkt)
    {
        if (buf->event->isRead())
        {
            Addr offset = evId >> 32;

            assert(offset + pkt->getSize() <= bufSize);

            memcpy(buf->bytes + offset,
                   pkt->getConstPtr<uint8_t>(),
                   pkt->getSize());
        }

        buf->event->freeSlots++;
    }

    // transfer done?
    if (buf->event->result != DtuError::NONE ||
        (buf->event->remaining == 0 &&
         buf->event->freeSlots == dtu.reqCount))
    {
        buf->event->transferDone(buf->event->result);

        DPRINTFS(DtuXfers, (&dtu), "buf%d: Transfer done\n",
                 buf->id);

        // we're done with this buffer now
        if (buf->event->isRead())
            reads.sample(dtu.curCycle() - buf->event->startCycle);
        else
            writes.sample(dtu.curCycle() - buf->event->startCycle);
        buf->event->finish();
        buf->event = NULL;

        // start the next one, if there is any
        if (!queue.empty())
        {
            TransferEvent *ev = queue.front();
            queue.pop_front();
            dtu.schedule(ev, dtu.clockEdge(Cycles(1)));
        }
    }
    else if(buf->event->remaining > 0)
        buf->event->process();
}

void
XferUnit::TransferEvent::abort(DtuError error)
{
    DPRINTFS(DtuXfers, (&xfer->dtu),
        "buf%d: aborting transfer (%d)\n",
        buf->id,
        static_cast<int>(error));

    buf->event->result = error;
    if (trans)
    {
        trans->abort();
        trans = NULL;
        // will be deleted by abort()
    }

    xfer->aborts++;

    if(scheduled())
        xfer->dtu.deschedule(this);

    buf->event->remaining = 0;
    xfer->recvMemResponse(id, NULL);
}

void
XferUnit::startTransfer(TransferEvent *event, Cycles delay)
{
    event->xfer = this;
    event->freeSlots = dtu.reqCount;
    event->startCycle = dtu.curCycle();

    if (event->isRead())
        bytesRead.sample(event->remaining);
    else
        bytesWritten.sample(event->remaining);

    dtu.schedule(event, dtu.clockEdge(Cycles(delay + 1)));

    // finish the noc request now to make the port unbusy
    if (event->isRemote())
        dtu.schedNocRequestFinished(dtu.clockEdge(Cycles(1)));
}

bool
XferUnit::abortTransfers(uint types)
{
    bool rem = false;

    for (size_t i = 0; i < bufCount; ++i)
    {
        auto ev = bufs[i]->event;
        if (!ev)
            continue;

        bool abort = !ev->isRemote() && (types & ABORT_LOCAL);
        // received messages are only aborted on reset
        abort &= !(ev->flags() & XferFlags::MSGRECV) || (types & ABORT_MSGS);

        if (abort)
        {
            ev->abort(DtuError::ABORT);
            // by default, we auto-delete it, but in this case, we have to do
            // that manually since it's not the current event
            delete ev;
        }
        else
            rem = true;
    }

    return !rem;
}

XferUnit::Buffer *
XferUnit::getBuffer(uint64_t evId)
{
    for (size_t i = 0; i < bufCount; ++i)
    {
        if (bufs[i]->event && bufs[i]->event->id == evId)
            return bufs[i];
    }

    return NULL;
}

XferUnit::Buffer*
XferUnit::allocateBuf(TransferEvent *event, uint flags)
{
    // don't allow message receives in parallel. because otherwise we run into race conditions.
    // e.g., we could overwrite unread messages because we can't increase the message counter when
    // the receive starts (to not notify SW) and thus might start receiving without having space
    // another problem is that we might finish receiving the second message before the first and
    // then increase the message counter, so that the SW looks at the first message, which is not
    // ready yet.
    if (flags & XferFlags::MSGRECV)
    {
        for (size_t i = 0; i < bufCount; ++i)
        {
            if (bufs[i]->event && (bufs[i]->event->flags() & XferFlags::MSGRECV))
                return NULL;
        }
    }

    // the first buffer cannot cause pagefaults; thus we can only use it if for
    // transfers which abort if a pagefault is caused
    // this is required to resolve a deadlock due to additional transfers that
    // handle a already running pagefault transfer.
    size_t i = !dtu.tlb() || (flags & XferFlags::NOPF) ? 0 : 1;
    for (; i < bufCount; ++i)
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
