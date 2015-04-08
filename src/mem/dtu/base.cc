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
      baseAddr(p->cpu_base_addr),
      spmPktSize(p->spm_pkt_size),
      nocPktSize(p->noc_pkt_size),
      state(State::IDLE),
      transmitManager(p),
      tickEvent(this)
{}

Tick
BaseDtu::handleCpuRequest(PacketPtr pkt)
{
    DPRINTF(Dtu, "Received %s request from CPU at address 0x%x\n",
            pkt->isWrite() ? "write" : "read",
            pkt->getAddr());

    Addr paddr = pkt->getAddr();
    paddr -= baseAddr; // from now on only work with the address offset
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

    RegFile::IntReg cmd = regFile.readReg(DtuRegister::COMMAND);
    if (state == State::IDLE && cmd != 0)
        startTransaction(cmd);

    // restore the original address
    paddr += baseAddr;
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
BaseDtu::startTransaction(RegFile::IntReg cmd)
{
    if (cmd == RECEIVE_CMD)
         state = State::RECEIVING;
    else if (cmd == TRANSMIT_CMD)
         state = State::TRANSMITTING;
    else
        panic("Unknown command");

    DPRINTF(Dtu, "Start transaction (%s)\n",
            state == State::RECEIVING ? "receiving" : "transmitting");

    regFile.setReg(DtuRegister::STATUS, BUSY_STATUS);

    TransmissionDescriptor transmission;
    transmission.sourceAddr = regFile.readReg(DtuRegister::SOURCE_ADDR);
    transmission.sourceCoreId = 0; // TODO
    transmission.targetAddr = regFile.readReg(DtuRegister::TARGET_ADDR);
    transmission.tagetCoreId = regFile.readReg(DtuRegister::TARGET_COREID);
    transmission.size = regFile.readReg(DtuRegister::SIZE);

    transmitManager.init(transmission);

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
        PacketPtr pkt = transmitManager.generateNewSpmRequest();

        if (pkt != nullptr && sendSpmRequest(pkt))
        {
            schedule(tickEvent, nextCycle());
        }
    }
}
