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
#include "mem/tcu/reg_file.hh"
#include "mem/tcu/tcu.hh"

namespace gem5
{
namespace tcu
{

int EpFile::EpCache::next_id = 1;

EpFile::EpCache::EpCache(EpFile &_epfile, EpCachePrio prio, const char *name)
    : state(FETCH), autoFinish(), autoDelete(), functional(), pending(), epRequests(), func(),
      cachedEps(), access(RegAccess::TCU), prio(prio), name(name), id(next_id), epfile(_epfile)
{
    next_id += 1;
}

void EpFile::EpCache::addEp(epid_t ep)
{
    cachedEps[ep].ep = Ep();
    cachedEps[ep].dirty = false;
}

Ep
EpFile::EpCache::getEp(epid_t ep)
{
    return cachedEps[ep].ep;
}

void
EpFile::EpCache::onFetched(std::function<void (EpCache&)> _func,
                           bool _autoFinish, bool _functional)
{
    func = _func;
    autoFinish = _autoFinish;
    functional = _functional;
    // note: we need to track the number of onFetched calls to support cases that acquire additional
    // EPs within the first onFetched callback (e.g., for replies). In such cases we want to keep
    // the lock until we are done with everything (and thus want to postpone the finish call until
    // that point).
    pending++;

    if (functional)
        process();
    else
        epfile.tcu.schedule(this, epfile.tcu.clockEdge(Cycles(1)));
}

void
EpFile::EpCache::onFinished(std::function<void (EpCache&)> _func, bool _functional)
{
    func = _func;
    autoFinish = false;
    functional = _functional;
    state = WRITEBACK;

    if (functional)
        process();
    else
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
EpFile::EpCache::handleEpResponse(epid_t ep, PacketPtr pkt)
{
    switch (state)
    {
        case WRITEBACK_WAIT:
        {
            DPRINTFS(TcuEps, (&epfile),
                    "cache[%s:%d]: writeback of EP%u completed\n", name, id, ep);
            
            RegFile::printEpAccess(epfile.tcu, cachedEps[ep].ep, false, access);

            cachedEps[ep].dirty = false;

            if (--epRequests == 0 && !functional)
                epfile.tcu.schedule(this, epfile.tcu.clockEdge(Cycles(1)));
            break;
        }

        case FETCH_WAIT:
        {
            DPRINTFS(TcuEps, (&epfile),
                    "cache[%s:%d]: fetch of EP%u completed\n", name, id, ep);

            RegFile::reg_t *regs = pkt->getPtr<RegFile::reg_t>();
            for(size_t i = 0; i < numEpRegs; ++i)
                cachedEps[ep].ep.inval.r[i] = regs[i];
            cachedEps[ep].ep.inval.id = ep;

            RegFile::printEpAccess(epfile.tcu, cachedEps[ep].ep, true, access);
            
            if (--epRequests == 0 && !functional)
                epfile.tcu.schedule(this, epfile.tcu.clockEdge(Cycles(1)));
            break;
        }

        default:
            panic("Received EP response in unexpected state: %d\n", state);
    }
}

void
EpFile::EpCache::process()
{
    if (state == WRITEBACK || state == FETCH)
    {
        // messaging is exclusive to prevent that message receptions and commands interfere with
        // each other. register access cannot happen in parallel either, because the CPU might issue
        // multiple register writes simultenaously, causing a race.
        if (prio != Memory)
        {
            if (!epfile.takeLock(this))
            {
                DPRINTFS(TcuEps, (&epfile),
                        "cache[%s:%d]: unable to take lock\n", name, id);
                panic_if(functional, "Lock taken with functional access!?");
                epfile.tcu.schedule(this, epfile.tcu.clockEdge(Cycles(1)));
                return;
            }
        }
    }

    switch (state)
    {
        case WRITEBACK:
            // change the state first, because in functional mode we call handleEpResponse
            // immediately.
            state = WRITEBACK_WAIT;

            // writeback all EPs that were changed
            for(auto it = cachedEps.begin(); it != cachedEps.end(); ++it)
            {
                if (it->second.dirty)
                {
                    auto epSize = numEpRegs * sizeof(RegFile::reg_t);
                    auto addr = epfile.tcu.regs().endpointsAddr() + it->first * epSize;
                    auto pkt = epfile.tcu.generateRequest(addr, epSize, MemCmd::WriteReq);
                    pkt->setData(reinterpret_cast<uint8_t*>(it->second.ep.inval.r));

                    DPRINTFS(TcuEps, (&epfile),
                            "cache[%s:%d]: writeback of EP%u to %#016x started\n",
                            name, id, it->first, addr);

                    epfile.tcu.sendEpRequest(pkt, this, it->first, Cycles(1), functional);
                    epRequests++;
                }
            }

            [[fallthrough]];

        case WRITEBACK_WAIT:
            if (epRequests == 0)
            {
                // here we're always finished
                finish();
                // notify the user afterwards
                func(*this);

                if (autoDelete)
                    setFlags(AutoDelete);
            }
            break;

        case FETCH:
            state = FETCH_WAIT;

            for(auto it = cachedEps.begin(); it != cachedEps.end(); ++it)
            {
                // if not already done, fetch the EP from the EP cache
                if (it->second.ep.type() == EpType::INVALID)
                {
                    auto epSize = numEpRegs * sizeof(RegFile::reg_t);
                    auto addr = epfile.tcu.regs().endpointsAddr() + it->first * epSize;
                    auto pkt = epfile.tcu.generateRequest(addr, epSize, MemCmd::ReadReq);

                    DPRINTFS(TcuEps, (&epfile),
                            "cache[%s:%d]: fetch of EP%u from %#016x started\n",
                            name, id, it->first, addr);

                    epfile.tcu.sendEpRequest(pkt, this, it->first, Cycles(1), functional);
                    epRequests++;
                }
            }

            [[fallthrough]];
    
        case FETCH_WAIT:
            if (epRequests == 0)
            {
                func(*this);

                // one operation done
                assert(pending > 0);
                pending--;
                // check if we're finished with this EpCache
                if (autoFinish && pending == 0)
                    finish();
                else if (state == FETCH_WAIT)
                    state = FETCH;

                if (autoDelete)
                    setFlags(AutoDelete);
            }
            break;
        
        default:
            panic("Unexpected state");
    }
}

EpFile::EpFile(Tcu &_tcu)
    : tcu(_tcu), lock()
{}

const std::string EpFile::name() const
{
    return tcu.name() + ".eps";
}

void
EpFile::handleEpResponse(EpCache *cache, epid_t ep, PacketPtr pkt)
{
    cache->handleEpResponse(ep, pkt);
}

bool
EpFile::takeLock(EpCache *cache)
{
    if (lock && lock != cache)
        return false;
    if (lock != cache)
    {
        DPRINTF(TcuEps, "cache[%s:%d]: taking lock\n", cache->name, cache->id);
        lock = cache;
    }
    return true;
}

void
EpFile::releaseLock(EpCache *cache)
{
    if (lock == cache)
    {
        DPRINTF(TcuEps, "cache[%s:%d]: freeing lock\n", cache->name, cache->id);
        lock = nullptr;
    }
}

EpFile::EpCache
EpFile::newCache(EpCachePrio type, const char *name)
{
    return EpCache(*this, type, name);
}

}
}
