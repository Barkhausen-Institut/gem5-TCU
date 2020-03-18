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

#ifndef __MEM_TCU_CORE_REQS_HH__
#define __MEM_TCU_CORE_REQS_HH__

#include "mem/tcu/regfile.hh"
#include "mem/tcu/xfer_unit.hh"

class Tcu;

class CoreRequests
{
  public:

    struct Request
    {
        enum Type
        {
            TRANSLATE,
            FOREIGN_RECV,
        };

        explicit Request(CoreRequests &_req, Type _type)
            : req(_req),
              type(_type),
              waiting(true)
        {}
        virtual ~Request() {
        }

        virtual void start(size_t id);
        virtual void complete(size_t id, RegFile::reg_t resp) = 0;

        CoreRequests &req;
        Type type;
        bool waiting;
    };

    struct XlateRequest : public Request
    {
        explicit XlateRequest(CoreRequests &req)
            : Request(req, TRANSLATE) {}

        void start(size_t id) override;
        void complete(size_t id, RegFile::reg_t resp) override;

        XferUnit::Translation *trans;
        uint16_t asid;
        Addr virt;
        uint access;
    };

    struct ForeignRecvRequest : public Request
    {
        explicit ForeignRecvRequest(CoreRequests &req)
            : Request(req, FOREIGN_RECV) {}

        void start(size_t id) override;
        void complete(size_t id, RegFile::reg_t resp) override;

        XferUnit::TransferEvent *event;
        epid_t epId;
        vpeid_t vpeId;
    };

    explicit CoreRequests(Tcu &tcu, size_t bufCount);

    const std::string name() const;

    void regStats();

    void startTranslate(size_t id,
                        vpeid_t vpeId,
                        Addr virt,
                        uint access,
                        XferUnit::Translation *trans);

    void startForeignReceive(size_t id,
                             epid_t epId,
                             vpeid_t vpeId,
                             XferUnit::TransferEvent *event);

    void completeReqs();

    void abortReq(size_t id);

  private:

    Tcu &tcu;
    Request **reqs;
    size_t reqSlots;

    Stats::Scalar coreReqs;
    Stats::Scalar coreDelays;
    Stats::Scalar coreFails;
};

#endif // __MEM_TCU_CORE_REQS_HH__
