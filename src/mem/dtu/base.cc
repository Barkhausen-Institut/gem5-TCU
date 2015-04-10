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

#include "base/trace.hh"
#include "debug/Dtu.hh"
#include "mem/dtu/base.hh"
#include "sim/system.hh"

BaseDtu::BaseDtu(const BaseDtuParams* p)
    : MemObject(p),
      regFile(p->name + ".regFile"),
      cpuBaseAddr(p->cpu_base_addr),
      nocAddrBits(p->noc_addr_bits),
      spmPktSize(p->spm_pkt_size),
      nocPktSize(p->noc_pkt_size),
      masterId(p->system->getMasterId(name())),
      state(State::IDLE),
      tickEvent(this),
      bytesRead(0)
{
    nocBaseAddr = getDtuBaseAddr(p->core_id);
}

Addr
BaseDtu::getDtuBaseAddr(unsigned coreId) const
{
    // XXX we assume 64 bit address width
    return static_cast<Addr>(coreId) << (64 - nocAddrBits);
}

void
BaseDtu::wakeUp()
{
    schedule(tickEvent, nextCycle());
}

PacketPtr
BaseDtu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
{
    Request::Flags flags;

    auto req = new Request(paddr, size, flags, masterId);

    auto pkt = new Packet(req, MemCmd::ReadReq);
    auto pktData = new uint8_t[size];
    pkt->dataDynamic(pktData);

    return pkt;
}

Tick
BaseDtu::handleCpuRequest(PacketPtr pkt)
{
    DPRINTF(Dtu, "Received %s request from CPU at address 0x%x\n",
            pkt->isWrite() ? "write" : "read",
            pkt->getAddr());

    Addr paddr = pkt->getAddr();
    paddr -= cpuBaseAddr; // from now on only work with the address offset
    pkt->setAddr(paddr);

    Tick delay = 0;

    if (pkt->isWrite() && state != State::IDLE)
        panic("Write requests while busy are forbidden!");

    /*
     * TODO The request is handled immediatly when arriving. However, we should
     *      pay for the delay caused by the transport layer (pkt->headerDelay
     *      and pkt->payloadDelay) first.
     */
    if (regFile.isRegisterAddr(paddr))
        delay = regFile.handleRequest(pkt);
    else
        // TODO generate an error response
        panic("Request at 0x%x failed as it is no valid register address", paddr);

    DtuReg cmd = regFile.readReg(DtuRegister::COMMAND);
    if (state == State::IDLE && cmd != 0)
        startTransaction(cmd);

    // restore the original address
    paddr += cpuBaseAddr;
    pkt->setAddr(paddr);

    Tick totalDelay = pkt->headerDelay + pkt->payloadDelay + delay;

    /*
     * The SimpleTimingPort already pays for the delay returned by recvAtomic
     *  -> reset the packet delay
     *
     * XXX I'm not sure if this is the right way to go. However, it seems
     *     better than simply ignoring the packet's delays as it is done for
     *     instance in SimpleMemory.
     */
    pkt->headerDelay  = 0;
    pkt->payloadDelay = 0;

    return totalDelay;
}

void
BaseDtu::startTransaction(DtuReg cmd)
{
    if (cmd == RECEIVE_CMD)
         state = State::RECEIVING;
    else if (cmd == TRANSMIT_CMD)
         state = State::TRANSMITTING;
    else
        panic("Unknown command");

    DPRINTF(Dtu, "Start transaction (%s)\n",
            state == State::RECEIVING ? "receiving" : "transmitting");

    bytesRead = 0;

    regFile.lock();
    regFile.setReg(DtuRegister::STATUS, BUSY_STATUS);

    schedule(tickEvent, nextCycle());
}

void
BaseDtu::completeSpmRequest(PacketPtr pkt)
{
    assert(state == State::RECEIVING || state == State::TRANSMITTING);

    if (state == State::TRANSMITTING)
    {
        Request* req = pkt->req;

        assert(pkt->isRead());
        assert(!pkt->isError());

        DPRINTF(Dtu, "Completing read from Scratchpad at address 0x%x\n",
                     req->getPaddr());

        // for now we just drop the packet
        delete req;
        delete pkt;
    }
    else
    {
        panic ("Receiving not yet implemented!");
    }
}

void
BaseDtu::tick()
{
    assert(state == State::RECEIVING || state == State::TRANSMITTING);

    if (state == State::RECEIVING)
    {
        panic("Receiving not yet implemented");
    }
    else
    {
        Addr messageSize =regFile.readReg(DtuRegister::SIZE);

        if (bytesRead < messageSize)
        {
            Addr pktSize = messageSize - bytesRead;

            if (pktSize > spmPktSize)
                pktSize = spmPktSize;

            Addr paddr = regFile.readReg(DtuRegister::SOURCE_ADDR) + bytesRead;

            bytesRead += pktSize;

            PacketPtr pkt = generateRequest(paddr, spmPktSize, MemCmd::ReadReq);

            // Only continue ticking when the packet was successfully sent.
            // Otherwise we sleep and wake up when a retry is received and the
            // packet was successfully resend. This procedure is handled by
            // sendSpmRequest.
            if (sendSpmRequest(pkt))
                schedule(tickEvent, nextCycle());
        }
    }
}
