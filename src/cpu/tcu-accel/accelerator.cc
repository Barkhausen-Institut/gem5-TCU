/*
 * Copyright (c) 2016, Nils Asmussen
 * Copyright (c) 2015, Christian Menard
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

#include "cpu/tcu-accel/accelerator.hh"
#include "debug/TcuConnector.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/reg_file.hh"
#include "sim/tile_memory.hh"

#include <iomanip>

const unsigned TcuAccel::EP_SYSS       = 8;
const unsigned TcuAccel::EP_SYSR       = 9;

bool
TcuAccel::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tcuaccel.completeRequest(pkt);
    return true;
}

void
TcuAccel::CpuPort::recvReqRetry()
{
    tcuaccel.recvRetry();
}

TcuAccel::TcuAccel(const TcuAccelParams &p)
  : ClockedObject(p),
    system(p.system),
    tickEvent(this),
    chunkSize(system->cacheLineSize()),
    maxDataSize(p.max_data_size),
    offset(p.offset),
    port("port", this),
    tcu(p.regfile_base_addr, system->getRequestorId(this, name()), p.id),
    id(p.id),
    atomic(system->isAtomicMode()),
    retryPkt(nullptr),
    connector()
{
    TileMemory *sys = dynamic_cast<TileMemory*>(system);
    haveVM = !sys->hasMem(id);
    // if we don't have VM, we have an SPM, which supports larger chunks
    if (!haveVM)
        chunkSize = maxDataSize;

    // kick things into action
    schedule(tickEvent, curTick());
}

Port &
TcuAccel::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return SimObject::getPort(if_name, idx);
}

bool
TcuAccel::sendPkt(PacketPtr pkt)
{
    if (atomic)
    {
        port.sendAtomic(pkt);
        completeRequest(pkt);
    }
    else if (!port.sendTimingReq(pkt))
    {
        retryPkt = pkt;
        return false;
    }

    return true;
}

void
TcuAccel::recvRetry()
{
    assert(retryPkt);
    if (port.sendTimingReq(retryPkt))
        retryPkt = nullptr;
}
