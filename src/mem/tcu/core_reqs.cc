/*
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

#include "debug/TcuCoreReqs.hh"
#include "mem/tcu/core_reqs.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/error.hh"

namespace gem5
{
namespace tcu
{

CoreRequests::CoreRequests(Tcu &_tcu, size_t bufCount)
    : tcu(_tcu),
      reqs()
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
    coreForeignRecvs
        .name(name() + ".coreForeignRecvs")
        .desc("Number of foreign-receive requests to the core");
    corePMPFailures
        .name(name() + ".corePMPFailures")
        .desc("Number of PMP failure requests to the core");
    coreDelays
        .name(name() + ".coreDelays")
        .desc("Number of delayed translate requests to the core");
    coreFails
        .name(name() + ".coreFails")
        .desc("Number of failed translate requests to the core");
}

void
CoreRequests::add(Request *req)
{
    if(reqs.size() == 1)
        req->start();
    else
        coreDelays++;
}

void
CoreRequests::startForeignReceive(epid_t epId,
                                  actid_t actId)
{
    size_t id = nextId();

    auto req = new ForeignRecvRequest(id, *this);
    req->epId = epId;
    req->actId = actId;
    reqs.push_back(req);

    DPRINTFS(TcuCoreReqs, (&tcu),
        "CoreRequest[%lu] = recvForeign(ep=%u, act=%u)\n",
        id, epId, actId);

    coreForeignRecvs++;
    add(req);
}

void
CoreRequests::startPMPFailure(Addr phys, bool write, TcuError error)
{
    if (!(tcu.regs().get(PrivReg::PRIV_CTRL) & PrivCtrl::PMP_FAILURES))
    {
        DPRINTFS(TcuCoreReqs, (&tcu),
            "Ignoring PMP-failure core request as PRIV_CTRL.PMP_FAILURES is disabled\n");
        return;
    }

    size_t id = nextId();

    auto req = new PMPFailureRequest(id, *this);
    req->phys = phys;
    req->write = write;
    req->error = error;
    reqs.push_back(req);

    DPRINTFS(TcuCoreReqs, (&tcu),
        "CoreRequest[%lu] = pmpFailure(phys=%#x, write=%d, error=%d)\n",
        id, phys, write, static_cast<int>(error));

    corePMPFailures++;
    add(req);
}

void
CoreRequests::Request::start()
{
    DPRINTFS(TcuCoreReqs, (&req.tcu), "CoreRequest[%lu] started\n", id);
    req.tcu.con().setIrq(BaseConnector::IRQ::CORE_REQ);
}

void
CoreRequests::ForeignRecvRequest::start()
{
    ForeignCoreReq freq = 0;
    freq.type = CoreMsgType::FOREIGN_REQ;
    freq.ep = epId;
    freq.act = actId;
    req.tcu.regs().set(PrivReg::CORE_REQ, freq);
    waiting = false;

    Request::start();
}

void
CoreRequests::PMPFailureRequest::start()
{
    PMPFailureCoreReq pmpreq = 0;
    pmpreq.type = CoreMsgType::PMP_FAILURE;
    pmpreq.phys = phys;
    pmpreq.write = write;
    pmpreq.error = static_cast<int>(error);
    req.tcu.regs().set(PrivReg::CORE_REQ, pmpreq);
    waiting = false;

    Request::start();
}

void
CoreRequests::completeReqs()
{
    CoreMsg resp = tcu.regs().get(PrivReg::CORE_REQ);
    assert(resp.type == CoreMsgType::RESP);
    assert(!reqs.empty());

    Request *req = reqs.front();
    DPRINTFS(TcuCoreReqs, (&tcu), "CoreRequest[%lu] done\n", req->id);
    reqs.pop_front();

    req->complete(resp);
    delete req;

    tcu.regs().set(PrivReg::CORE_REQ, CoreMsgType::IDLE);

    startNextReq();
}

void
CoreRequests::abortReq(size_t id)
{
    for (auto r = reqs.begin(); r != reqs.end(); ++r)
    {
        if ((*r)->id == id)
        {
            DPRINTFS(TcuCoreReqs, (&tcu), "CoreRequest[%lu] aborted\n", id);
            if (!(*r)->waiting)
               tcu.regs().set(PrivReg::CORE_REQ, CoreMsgType::IDLE);
           reqs.erase(r);
           delete *r;
           break;
        }
    }

    startNextReq();
}

void
CoreRequests::startNextReq()
{
    if (!reqs.empty())
    {
        Request *req = reqs.front();
        req->start();
    }
}

CoreRequests::Request *
CoreRequests::getById(size_t id) const
{
    for (auto r : reqs)
    {
        if (r->id == id)
            return r;
    }
    return nullptr;
}

size_t
CoreRequests::nextId() const
{
    for (size_t id = 0; ; ++id)
    {
        if (getById(id) == nullptr)
            return id;
    }
}

}
}
