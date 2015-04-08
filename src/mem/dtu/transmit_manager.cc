/*
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

#include "mem/dtu/transmit_manager.hh"
#include "sim/system.hh"
#include "debug/Dtu.hh"

TransmitManager::TransmitManager(const BaseDtuParams* p)
    : spmPktSize(p->spm_pkt_size),
      nocPktSize(p->noc_pkt_size),
      masterId(p->system->getMasterId(p->name))
{}

void
TransmitManager::init(TransmissionDescriptor _transmission)
{
    transmission = _transmission;

    currentReadAddr = _transmission.sourceAddr;
}

PacketPtr
TransmitManager::generateNewSpmRequest()
{
    Addr bytesRead = currentReadAddr - transmission.sourceAddr;

    if (bytesRead >= transmission.size)
        return nullptr;

    Addr pktSize = transmission.size - bytesRead;

    if (pktSize > spmPktSize)
        pktSize = spmPktSize;

    Request::Flags flags;

    auto req = new Request(currentReadAddr, pktSize, flags, masterId);
    //req->setThreadContext(transmission.sourceCoreId, 0);

    auto pkt = new Packet(req, MemCmd::ReadReq);
    auto pktData = new uint8_t[pktSize];
    pkt->dataDynamic(pktData);

    currentReadAddr += pktSize;

    return pkt;
}

PacketPtr
TransmitManager::generateNewNocRequest()
{
    panic("TransmitManager::generateNewNocRequest not yet implemented");
}
