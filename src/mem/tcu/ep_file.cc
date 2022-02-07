/*
 * Copyright (C) 2020 Nils Asmussen, Barkhausen Institut
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

#include "debug/TcuEps.hh"
#include "mem/tcu/ep_file.hh"
#include "mem/tcu/tcu.hh"

EpFile::EpCache::EpCache(EpFile &_epfile)
    : state(FETCH), autoFinish(), pending(), func(),
      cachedEps(), epfile(_epfile)
{}

void EpFile::EpCache::addEp(epid_t ep)
{
    cachedEps[ep].ep = Ep();
    cachedEps[ep].dirty = false;
}

Ep
EpFile::EpCache::getEp(epid_t ep)
{
    assert(epfile.lock == this);
    return cachedEps[ep].ep;
}

void
EpFile::EpCache::onFetched(std::function<void (EpCache&)> _func,
                             bool _autoFinish)
{
    func = _func;
    autoFinish = _autoFinish;
    pending++;

    epfile.tcu.schedule(this, epfile.tcu.clockEdge(Cycles(1)));
}

void
EpFile::EpCache::onFinished(std::function<void (EpCache&)> _func)
{
    func = _func;
    autoFinish = false;
    state = WRITEBACK;

    epfile.tcu.schedule(this, epfile.tcu.clockEdge(Cycles(1)));
}

void
EpFile::EpCache::finish()
{
    // everything should be written back by now
    for(auto it = cachedEps.begin(); it != cachedEps.end(); ++it)
        assert(!it->second.dirty);

    cachedEps.clear();
    pending = 0;
    state = FETCH;

    epfile.releaseLock(this);
}

void
EpFile::EpCache::process()
{
    if (state == WRITEBACK)
    {
        assert(epfile.lock == this);

        // writeback all EPs that were changed
        for(auto it = cachedEps.begin(); it != cachedEps.end(); ++it)
        {
            if (it->second.dirty)
            {
                DPRINTFS(TcuEps, (&epfile),
                         "cache[%#x]: writeback EP%u\n", this, it->first);
                epfile.tcu.regs().updateEp(it->second.ep.send);
                it->second.dirty = false;
            }
        }

        // here we're always finished
        finish();
        // notify the user afterwards
        func(*this);
    }
    else
    {
        // we need to take a lock while holding the EPs, because otherwise
        // message receptions could interfere with each other and with an
        // ongoing command.
        if (!epfile.takeLock(this))
        {
            DPRINTFS(TcuEps, (&epfile),
                     "cache[%#x]: unable to take lock\n", this);
            epfile.tcu.schedule(this, epfile.tcu.clockEdge(Cycles(1)));
            return;
        }

        for(auto it = cachedEps.begin(); it != cachedEps.end(); ++it)
        {
            // if not already done, fetch the EP from the register file
            if (it->second.ep.type() == EpType::INVALID)
            {
                DPRINTFS(TcuEps, (&epfile),
                         "cache[%#x]: fetching EP%u\n", this, it->first);
                it->second.ep = epfile.tcu.regs().getEp(it->first);
            }
        }

        func(*this);

        // one operation done
        assert(pending > 0);
        pending--;
        // check if we're finished with this EpCache
        if (autoFinish && pending == 0)
            finish();
    }
}

EpFile::EpFile(Tcu &_tcu)
    : tcu(_tcu), lock()
{}

const std::string EpFile::name() const
{
    return tcu.name() + ".eps";
}

bool
EpFile::takeLock(EpCache *cache)
{
    if (lock && lock != cache)
        return false;
    DPRINTF(TcuEps, "cache[%#x]: taking lock\n", cache);
    lock = cache;
    return true;
}

void
EpFile::releaseLock(EpCache *cache)
{
    DPRINTF(TcuEps, "cache[%#x]: freeing lock\n", cache);
    assert(lock == cache);
    lock = nullptr;
}

EpFile::EpCache
EpFile::newCache()
{
    return EpCache(*this);
}
