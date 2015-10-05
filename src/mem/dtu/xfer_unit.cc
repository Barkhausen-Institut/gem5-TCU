/*
 * Copyright (c) 2015, Christian Menard
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

#include "debug/Dtu.hh"
#include "debug/DtuBuf.hh"
#include "debug/DtuDetail.hh"
#include "debug/DtuPackets.hh"
#include "debug/DtuSysCalls.hh"
#include "debug/DtuPower.hh"
#include "mem/dtu/xfer_unit.hh"

void
XferUnit::TransferEvent::process()
{
    assert((trans.sourceAddr & (blockSize - 1)) == 0);

    if(trans.type != Dtu::NocPacketType::READ_REQ)
    {
        auto pkt = xfer.dtu.generateRequest(trans.sourceAddr, blockSize, MemCmd::ReadReq);

        xfer.dtu.sendSpmRequest(pkt,
                                -1,
                                xfer.dtu.transferToSpmRequestLatency,
                                Dtu::SpmPacketType::TRANSFER_REQUEST,
                                false);
    }
    else
    {
        xfer.forwardToNoc(NULL, 0, 0);
    }
}

void
XferUnit::startTransfer(Dtu::NocPacketType type,
                        NocAddr targetAddr,
                        Addr sourceAddr,
                        Addr size)
{
    assert(transferEvent.trans.size == 0);

    transferEvent.trans.targetAddr = targetAddr;
    transferEvent.trans.sourceAddr = sourceAddr;
    transferEvent.trans.size = size;
    transferEvent.trans.type = type;

    dtu.schedule(transferEvent, dtu.clockEdge(Cycles(1)));
}

void
XferUnit::sendToNoc(Dtu::NocPacketType type,
                    NocAddr targetAddr,
                    const void* data,
                    Addr size,
                    Tick spmPktHeaderDelay,
                    Tick spmPktPayloadDelay)
{
    auto cmd = type == Dtu::NocPacketType::READ_REQ ? MemCmd::ReadReq : MemCmd::WriteReq;
    auto pkt = dtu.generateRequest(targetAddr.getAddr(),
                                   size,
                                   cmd);
    if(data)
    {
        memcpy(pkt->getPtr<uint8_t>(),
               data,
               size);
    }

    /*
     * See sendNocMessage() for an explanation of delay handling.
     */
    Cycles delay = dtu.spmResponseToNocRequestLatency;
    delay += dtu.ticksToCycles(spmPktHeaderDelay);
    pkt->payloadDelay = spmPktPayloadDelay;
    dtu.printPacket(pkt);
    dtu.sendNocRequest(type, pkt, delay);
}

void
XferUnit::forwardToNoc(const void* data,
                       Tick spmPktHeaderDelay,
                       Tick spmPktPayloadDelay)
{
    Transfer &trans = transferEvent.trans;

    Addr requestSize = std::min(trans.size, blockSize);

    trans.targetAddr.last = trans.size == requestSize;

    sendToNoc(trans.type,
              trans.targetAddr,
              reinterpret_cast<const uint8_t*>(data),
              requestSize,
              spmPktHeaderDelay,
              spmPktPayloadDelay);

    /*
     * to next cacheline
     */
    trans.sourceAddr += requestSize;
    trans.targetAddr.offset += requestSize;
    trans.size -= requestSize;

    if (trans.size > 0)
        dtu.schedule(transferEvent, dtu.clockEdge(Cycles(1)));
}
