/*
 * Copyright (c) 2019, Nils Asmussen
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

#include "debug/TcuCoreReqs.hh"
#include "mem/tcu/core_reqs.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/error.hh"

CoreRequests::CoreRequests(Tcu &_tcu, size_t bufCount)
    : tcu(_tcu),
      reqs(new Request*[bufCount + 1]()),
      reqSlots(bufCount + 1)
{
}

const std::string
CoreRequests::name() const
{
    return tcu.name();
}

void
CoreRequests::regStats()
{
    coreReqs
        .name(name() + ".coreReqs")
        .desc("Number of translate requests to the core");
    coreDelays
        .name(name() + ".coreDelays")
        .desc("Number of delayed translate requests to the core");
    coreFails
        .name(name() + ".coreFails")
        .desc("Number of failed translate requests to the core");
}

void
CoreRequests::startTranslate(size_t id,
                             unsigned vpeId,
                             Addr virt,
                             uint access,
                             XferUnit::Translation *trans)
{
    if (reqs[id] != nullptr)
        return;

    auto req = new XlateRequest(*this);
    req->trans = trans;
    req->virt = virt;
    req->asid = vpeId;
    req->access = access;
    reqs[id] = req;

    DPRINTFS(TcuCoreReqs, (&tcu),
        "CoreRequest[%lu] = translate(asid=%#x, addr=%p, acc=%#x)\n",
        id, req->asid, virt, access);
    coreReqs++;

    if(tcu.regs().get(PrivReg::CORE_REQ) == 0)
        req->start(id);
    else
        coreDelays++;
}

void
CoreRequests::Request::start(size_t id)
{
    DPRINTFS(TcuCoreReqs, (&req.tcu), "CoreRequest[%lu] started\n", id);
    req.tcu.setIrq();
}

void
CoreRequests::XlateRequest::start(size_t id)
{
    const Addr mask = TcuTlb::PAGE_MASK;
    const Addr virtPage = virt & ~mask;
    const Addr val = (static_cast<Addr>(asid) << 48) | virtPage |
                     (access << 1) | (id << 6);
    req.tcu.regs().set(PrivReg::CORE_REQ, val);
    waiting = false;

    Request::start(id);
}

void
CoreRequests::XlateRequest::complete(size_t id, RegFile::reg_t resp)
{
    Addr mask = (resp & TcuTlb::LARGE) ? TcuTlb::LPAGE_MASK
                                       : TcuTlb::PAGE_MASK;

    NocAddr phys((resp & ~mask) | (virt & mask));
    uint flags = resp & (TcuTlb::RWX | TcuTlb::LARGE | TcuTlb::FIXED);
    if (flags == 0)
    {
        trans->finished(false, phys);
        req.coreFails++;
    }
    else
    {
        req.tcu.tlb()->insert(virt, asid, phys, flags);
        trans->finished(true, phys);
    }
}

void
CoreRequests::startForeignReceive(size_t id,
                                  unsigned epId,
                                  unsigned vpeId,
                                  XferUnit::TransferEvent *event)
{
    if (reqs[id] != nullptr)
        return;

    auto req = new ForeignRecvRequest(*this);
    req->event = event;
    req->epId = epId;
    req->vpeId = vpeId;
    reqs[id] = req;

    DPRINTFS(TcuCoreReqs, (&tcu), "CoreRequest[%lu] = recvForeign(ep=%u, vpe=%u)\n",
        id, epId, vpeId);
    coreReqs++;

    if (tcu.regs().get(PrivReg::CORE_REQ) == 0)
        req->start(id);
    else
        coreDelays++;
}

void
CoreRequests::ForeignRecvRequest::start(size_t id)
{
    auto val = (epId << 28) | (vpeId << 12) | (id << 6) | 1;
    req.tcu.regs().set(PrivReg::CORE_REQ, val);
    waiting = false;

    Request::start(id);
}

void
CoreRequests::ForeignRecvRequest::complete(size_t, RegFile::reg_t)
{
    event->start();
}

void
CoreRequests::completeReqs()
{
    RegFile::reg_t resp = tcu.regs().get(PrivReg::CORE_RESP);
    if (resp)
    {
        size_t id = (resp >> 6) & 0x7;
        assert(reqs[id] != nullptr);
        DPRINTFS(TcuCoreReqs, (&tcu), "CoreRequest[%lu] done\n", id);

        reqs[id]->complete(id, resp);
        delete reqs[id];
        reqs[id] = nullptr;

        tcu.regs().set(PrivReg::CORE_RESP, 0);
    }

    if (tcu.regs().get(PrivReg::CORE_REQ) == 0)
    {
        for (size_t id = 0; id < reqSlots; ++id)
        {
            if (reqs[id] && reqs[id]->waiting)
            {
                reqs[id]->start(id);
                break;
            }
        }
    }
}

void
CoreRequests::abortReq(size_t id)
{
    if (reqs[id] && !reqs[id]->waiting)
        tcu.regs().set(PrivReg::CORE_REQ, 0);
    delete reqs[id];
    reqs[id] = nullptr;
}
