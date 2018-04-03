/*
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

#include "mem/probes/mem_watch.hh"

#include "base/trace.hh"
#include "params/MemWatchProbe.hh"

MemWatchProbe::MemWatchProbe(MemWatchProbeParams *p)
    : BaseMemProbe(p),
      ranges(p->ranges)
{
}

void
MemWatchProbe::handleRequest(const ProbePoints::PacketInfo &pkt_info)
{
    // ignore the request if we don't know the virtual address
    if(pkt_info.virt == -1)
        return;

    AddrRange pkt_range(pkt_info.virt, pkt_info.virt + pkt_info.size - 1);
    for(auto &&range : ranges)
    {
        if (range.intersects(pkt_range))
        {
            Trace::getDebugLogger()->dprintf(curTick(), name(),
                "%s access to %p..%p (watching %p..%p):\n",
                pkt_info.cmd.isRead() ? "rd" : "wr",
                pkt_range.start(), pkt_range.end(),
                range.start(), range.end());
            Trace::getDebugLogger()->dump(
                curTick(), name(), pkt_info.data, pkt_info.size);
        }
    }
}


MemWatchProbe *
MemWatchProbeParams::create()
{
    return new MemWatchProbe(this);
}
