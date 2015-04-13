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
      maxPktSize(p->max_pkt_size),
      masterId(p->system->getMasterId(name())),
      state(State::IDLE),
      tickEvent(this)
{
    nocBaseAddr = getDtuBaseAddr(p->core_id);
}

Addr
BaseDtu::getDtuBaseAddr(unsigned coreId) const
{
    // XXX we assume 64 bit address width
    return static_cast<Addr>(coreId) << (64 - nocAddrBits);
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

bool
BaseDtu::canHandleNocRequest(PacketPtr pkt)
{
    // TODO
    return true;
}

Tick
BaseDtu::handleNocRequest(PacketPtr pkt)
{
    panic("handleNocRequest() not yet implemented!");

    // TODO
    return 0;
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

    readAddr = regFile.readReg(DtuRegister::SOURCE_ADDR);
    writeAddr = regFile.readReg(DtuRegister::TARGET_ADDR);

    regFile.lock();
    regFile.setReg(DtuRegister::STATUS, BUSY_STATUS);

    schedule(tickEvent, nextCycle());
}

void
BaseDtu::completeSpmRequest(PacketPtr pkt)
{
    assert(state == State::RECEIVING || state == State::TRANSMITTING);
    assert(!dynamic_cast<SenderState*>(pkt->popSenderState())->isNocRequest);

    if (state == State::TRANSMITTING)
    {
        Request* req = pkt->req;

        assert(pkt->isRead());
        assert(!pkt->isError());

        DPRINTF(Dtu, "Completing read from Scratchpad at address 0x%x\n",
                     req->getPaddr());

        // Fill the buffer. We assume that all packets arrive in order.
        pktBuffer.push(pkt);
    }
    else
    {
        panic ("Receiving not yet implemented!");
    }
}

void
BaseDtu::completeNocRequest(PacketPtr pkt)
{
    assert(state == State::RECEIVING || state == State::TRANSMITTING);

    if (state == State::TRANSMITTING)
    {
        Request* req = pkt->req;

        assert(pkt->isWrite());
        assert(!pkt->isError());

        DPRINTF(Dtu, "Completing write to NoC at address 0x%x\n",
                     req->getPaddr());

        // clean up
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
        Addr messageSize = regFile.readReg(DtuRegister::SIZE);

        Addr bytesRead = readAddr - regFile.readReg(DtuRegister::SOURCE_ADDR);

        /*
         * TODO Limit buffer size. Send packats only when there is a free slot
         *      in the packet buffer (pktBuffer)
         */

        if (bytesRead < messageSize && isSpmPortReady())
        {
            Addr pktSize = messageSize - bytesRead;

            if (pktSize > maxPktSize)
                pktSize = maxPktSize;

            PacketPtr pkt = generateRequest(readAddr, pktSize, MemCmd::ReadReq);

            readAddr += pktSize;

            auto state = new SenderState(false);
            pkt->pushSenderState(state);

            sendSpmRequest(pkt);
        }

        if (!pktBuffer.empty() && isNocPortReady())
        {
            // the buffer contains responses from Scratchpad read requests
            PacketPtr spmPkt = pktBuffer.front();
            pktBuffer.pop();

            Addr paddr = getDtuBaseAddr(regFile.readReg(DtuRegister::TARGET_COREID));

            Addr pktSize = spmPkt->getSize();

            paddr += writeAddr;

            writeAddr += pktSize;

            PacketPtr pkt = generateRequest(paddr, pktSize, MemCmd::WriteReq);

            memcpy(pkt->getPtr<uint8_t>(), spmPkt->getPtr<uint8_t>(), spmPkt->getSize());

            sendNocRequest(pkt);

            delete spmPkt->req;
            delete spmPkt;
        }
    }

    schedule(tickEvent, nextCycle());
}
