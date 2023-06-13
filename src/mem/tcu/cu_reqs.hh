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

#ifndef __MEM_TCU_CU_REQS_HH__
#define __MEM_TCU_CU_REQS_HH__

#include "mem/tcu/reg_file.hh"
#include "mem/tcu/xfer_unit.hh"

#include <list>

namespace gem5
{
namespace tcu
{

class Tcu;

class CURequests
{
  public:

    struct Request
    {
        explicit Request(size_t _id, CURequests &_req)
            : id(_id),
              req(_req),
              waiting(true)
        {}
        virtual ~Request() {
        }

        virtual void start();
        virtual void complete(RegFile::reg_t resp) = 0;

        size_t id;
        CURequests &req;
        bool waiting;
    };

    struct ForeignRecvRequest : public Request
    {
        explicit ForeignRecvRequest(size_t id, CURequests &req)
            : Request(id, req) {}

        void start() override;
        void complete(RegFile::reg_t) override {}

        epid_t epId;
        actid_t actId;
    };

    struct PMPFailureRequest : public Request
    {
        explicit PMPFailureRequest(size_t id, CURequests &req)
            : Request(id, req) {}

        void start() override;
        void complete(RegFile::reg_t) override {}

        Addr phys;
        bool write;
        TcuError error;
    };

    explicit CURequests(Tcu &tcu, size_t bufCount);

    const std::string name() const;

    void regStats();

    void startForeignReceive(epid_t epId,
                               actid_t actId);
    void startPMPFailure(Addr phys, bool write, TcuError error);

    void completeReqs();

    void abortReq(size_t id);

  private:

    void add(Request *req);
    void startNextReq();
    Request *getById(size_t id) const;
    size_t nextId() const;

    Tcu &tcu;
    std::list<Request*> reqs;

    statistics::Scalar cuForeignRecvs;
    statistics::Scalar cuPMPFailures;
    statistics::Scalar cuDelays;
    statistics::Scalar cuFails;
};

}
}

#endif // __MEM_TCU_CU_REQS_HH__
