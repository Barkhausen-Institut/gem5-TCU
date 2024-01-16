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

#include "debug/TcuCUReqs.hh"
#include "mem/tcu/cu_reqs.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/error.hh"

namespace gem5
{
namespace tcu
{

CURequests::CURequests(Tcu &_tcu, size_t bufCount)
    : tcu(_tcu),
      reqs()
{
}

const std::string
CURequests::name() const
{
    return tcu.name();
}

void
CURequests::regStats()
{
    cuForeignRecvs
        .name(name() + ".cuReqForeignRecvs")
        .desc("Number of foreign-receive requests to the CU");
    cuPMPFailures
        .name(name() + ".cuReqPMPFailures")
        .desc("Number of PMP failure requests to the CU");
    cuDelays
        .name(name() + ".cuReqDelays")
        .desc("Number of delayed translate requests to the CU");
    cuFails
        .name(name() + ".cuReqFails")
        .desc("Number of failed translate requests to the CU");
}

void
CURequests::reset()
{
    while(!reqs.empty())
    {
        Request *r = reqs.front();
        DPRINTFS(TcuCUReqs, (&tcu), "CURequest[%lu] aborted\n", r->id);
        delete r;
        reqs.pop_front();
    }

    tcu.regs().set(PrivReg::CU_REQ, CUMsgType::IDLE);
}

void
CURequests::add(Request *req)
{
    if(reqs.size() == 1)
        req->start();
    else
        cuDelays++;
}

void
CURequests::startForeignReceive(epid_t epId,
                                  actid_t actId)
{
    size_t id = nextId();

    auto req = new ForeignRecvRequest(id, *this);
    req->epId = epId;
    req->actId = actId;
    reqs.push_back(req);

    DPRINTFS(TcuCUReqs, (&tcu),
        "CURequest[%lu] = recvForeign(ep=%u, act=%u)\n",
        id, epId, actId);

    cuForeignRecvs++;
    add(req);
}

void
CURequests::startPMPFailure(Addr phys, bool write, TcuError error)
{
    if (!(tcu.regs().get(PrivReg::PRIV_CTRL) & PrivCtrl::PMP_FAILURES))
    {
        DPRINTFS(TcuCUReqs, (&tcu),
            "Ignoring PMP-failure CU request as PRIV_CTRL.PMP_FAILURES is disabled\n");
        return;
    }

    size_t id = nextId();

    auto req = new PMPFailureRequest(id, *this);
    req->phys = phys;
    req->write = write;
    req->error = error;
    reqs.push_back(req);

    DPRINTFS(TcuCUReqs, (&tcu),
        "CURequest[%lu] = pmpFailure(phys=%#x, write=%d, error=%d)\n",
        id, phys, write, static_cast<int>(error));

    cuPMPFailures++;
    add(req);
}

void
CURequests::Request::start()
{
    DPRINTFS(TcuCUReqs, (&req.tcu), "CURequest[%lu] started\n", id);
    req.tcu.con().setIrq(BaseConnector::IRQ::CU_REQ);
}

void
CURequests::ForeignRecvRequest::start()
{
    ForeignCUReq freq = 0;
    freq.type = CUMsgType::FOREIGN_REQ;
    freq.ep = epId;
    freq.act = actId;
    req.tcu.regs().set(PrivReg::CU_REQ, freq);
    waiting = false;

    Request::start();
}

void
CURequests::PMPFailureRequest::start()
{
    PMPFailureCUReq pmpreq = 0;
    pmpreq.type = CUMsgType::PMP_FAILURE;
    pmpreq.phys = phys;
    pmpreq.write = write;
    pmpreq.error = static_cast<int>(error);
    req.tcu.regs().set(PrivReg::CU_REQ, pmpreq);
    waiting = false;

    Request::start();
}

void
CURequests::completeReqs()
{
    CUMsg resp = tcu.regs().get(PrivReg::CU_REQ);
    assert(resp.type == CUMsgType::RESP);
    assert(!reqs.empty());

    Request *req = reqs.front();
    DPRINTFS(TcuCUReqs, (&tcu), "CURequest[%lu] done\n", req->id);
    reqs.pop_front();

    req->complete(resp);
    delete req;

    tcu.regs().set(PrivReg::CU_REQ, CUMsgType::IDLE);

    startNextReq();
}

void
CURequests::abortReq(size_t id)
{
    for (auto r = reqs.begin(); r != reqs.end(); ++r)
    {
        if ((*r)->id == id)
        {
            DPRINTFS(TcuCUReqs, (&tcu), "CURequest[%lu] aborted\n", id);
            if (!(*r)->waiting)
               tcu.regs().set(PrivReg::CU_REQ, CUMsgType::IDLE);
           reqs.erase(r);
           delete *r;
           break;
        }
    }

    startNextReq();
}

void
CURequests::startNextReq()
{
    if (!reqs.empty())
    {
        Request *req = reqs.front();
        req->start();
    }
}

CURequests::Request *
CURequests::getById(size_t id) const
{
    for (auto r : reqs)
    {
        if (r->id == id)
            return r;
    }
    return nullptr;
}

size_t
CURequests::nextId() const
{
    for (size_t id = 0; ; ++id)
    {
        if (getById(id) == nullptr)
            return id;
    }
}

}
}
