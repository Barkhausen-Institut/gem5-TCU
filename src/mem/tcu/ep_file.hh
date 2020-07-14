/*
 * Copyright (c) 2020, Nils Asmussen
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

#ifndef __MEM_TCU_EP_FILE_HH__
#define __MEM_TCU_EP_FILE_HH__

#include "mem/tcu/reg_file.hh"
#include "mem/tcu/xfer_unit.hh"

#include <functional>
#include <map>
#include <vector>

class Tcu;

class EpFile
{
  public:

    class EpCache : public Event
    {
        struct CachedEp
        {
            Ep ep;
            bool dirty;
        };

        enum State
        {
            FETCH,
            WRITEBACK,
        };

      public:

        explicit EpCache(EpFile &eps);

        void addEp(epid_t ep);

        Ep getEp(epid_t ep);

        template<class T>
        void updateEp(const T &ep)
        {
            assert(epfile.lock == this);
            CachedEp &old = cachedEps[ep.id];
            old.ep.inval.id = ep.id;
            old.ep.inval.r[0] = ep.r0;
            old.ep.inval.r[1] = ep.r1;
            old.ep.inval.r[2] = ep.r2;
            old.dirty = true;
        }

        void setAutoFinish(bool _autoFinish) { autoFinish = _autoFinish; }

        void onFetched(std::function<void (EpCache&)> func,
                       bool autoFinish = true);

        void onFinished(std::function<void (EpCache&)> func);

        void finish();

      private:

        void process() override;

        const char* description() const override { return "EpCacheEvent"; }

        State state;

        bool autoFinish;

        int pending;

        std::function<void (EpCache&)> func;

        std::map<epid_t, CachedEp> cachedEps;

        EpFile &epfile;
    };

    explicit EpFile(Tcu &tcu);

    const std::string name() const;

    EpCache newCache();

  private:

    bool takeLock(EpCache *cache);

    void releaseLock(EpCache *cache);

    Tcu &tcu;

    EpCache *lock;
};

#endif // __MEM_TCU_EP_FILE_HH__
