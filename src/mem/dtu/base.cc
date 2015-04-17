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
      atomic(p->system->isAtomicMode()),
      regFile(p->name + ".regFile"),
      cpuBaseAddr(p->cpu_base_addr),
      nocAddrBits(p->noc_addr_bits),
      maxPktSize(p->max_pkt_size),
      masterId(p->system->getMasterId(name())),
      latency(p->latency),
      state(State::IDLE),
      tickEvent(*this)
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

    auto pkt = new Packet(req, cmd);
    auto pktData = new uint8_t[size];
    pkt->dataDynamic(pktData);

    return pkt;
}

bool
BaseDtu::handleCpuRequest(PacketPtr pkt)
{
    DPRINTF(Dtu, "Received %s request from CPU at address 0x%x\n",
            pkt->isWrite() ? "write" : "read",
            pkt->getAddr());

    Addr addr = pkt->getAddr();
    addr -= cpuBaseAddr; // from now on only work with the address offset
    pkt->setAddr(addr);

    if (pkt->isWrite() && state != State::IDLE)
        panic("Write requests while busy are forbidden!");

    if (regFile.isRegisterAddr(addr))
        regFile.handleRequest(pkt);
    else
        // TODO generate an error response
        panic("Request at 0x%x failed as it is no valid register address", addr);

    DtuReg cmd = regFile.readReg(DtuRegister::COMMAND);
    if (state == State::IDLE && cmd != 0)
        startTransaction(cmd);

    // restore the original address
    addr += cpuBaseAddr;
    pkt->setAddr(addr);

    if (!atomic)
        sendCpuResponse(pkt, latency);

    return true;
}

bool
BaseDtu::handleNocRequest(PacketPtr pkt)
{
    if (atomic && !isSpmPortReady())
        return false;

    Addr addr = pkt->getAddr();

    DPRINTF(Dtu, "Received %s request from NoC at address 0x%x\n",
            pkt->isWrite() ? "write" : "read",
            addr);

    addr -= nocBaseAddr;

    pkt->setAddr(addr);

    auto senderState = new SenderState(true);
    pkt->pushSenderState(senderState);

    DPRINTF(Dtu, "Forward request to Scratchpad at address 0x%x\n",
                 addr);

    sendSpmRequest(pkt);

    return true;
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

    regFile.setReg(DtuRegister::STATUS, BUSY_STATUS);
    regFile.lock();
}

void
BaseDtu::finishTransaction()
{
    assert(state != State::IDLE);

    DPRINTF(Dtu, "Transaction finished\n");

    state = State::IDLE;

    regFile.unlock();

    regFile.setReg(DtuRegister::STATUS, IDLE_STATUS);
    regFile.setReg(DtuRegister::COMMAND, 0);
}

void
BaseDtu::completeSpmRequest(PacketPtr pkt)
{
    DPRINTF(Dtu, "Completing %s Scratchpad at address 0x%x\n",
                  pkt->isRead() ? "read from" : "write to",
                  pkt->getAddr());

    auto senderState = dynamic_cast<SenderState*>(pkt->popSenderState());

    if (senderState->isNocRequest)
    {
        // Packet was generated by a remote DTU and was forwarded to the local SPM

        Addr addr = pkt->getAddr();

        // restore the original address
        addr += nocBaseAddr;

        pkt->setAddr(addr);

        // We only need to send a response in non atomic mode
        if (!atomic)
        {
            DPRINTF(Dtu, "Send response back to the NoC\n");

            sendNocResponse(pkt, latency);
        }

        // clean up
        delete senderState;
    }
    else
    {
        assert(state == State::RECEIVING || state == State::TRANSMITTING);

        // Packet was generated by this DTU
        if (state == State::TRANSMITTING)
        {
            assert(pkt->isRead());
            assert(!pkt->isError());

            pkt->pushSenderState(senderState);
            // Fill the buffer. We assume that all packets arrive in order.
            pktBuffer.push(pkt);
        }
        else // state == State::RECEIVING
        {
            assert(pkt->isWrite());
            assert(!pkt->isError());

            if (senderState->isLastRequest)
            {
                finishTransaction();
            }

            // clean up
            delete senderState;
            delete pkt->req;
            delete pkt;
        }
    }
}

void
BaseDtu::completeNocRequest(PacketPtr pkt)
{
    assert(state == State::RECEIVING || state == State::TRANSMITTING);

    if (state == State::TRANSMITTING)
    {
        assert(pkt->isWrite());
        assert(!pkt->isError());

        DPRINTF(Dtu, "Completing write to NoC at address 0x%x\n",
                     pkt->getAddr());

        auto senderState = dynamic_cast<BaseDtu::SenderState*>(pkt->popSenderState());

        if (senderState->isLastRequest)
        {
            finishTransaction();
        }

        // clean upi
        delete senderState;
        delete pkt->req;
        delete pkt;
    }
    else // state == State::RECEIVING
    {
        DPRINTF(Dtu, "Completing read from NoC at address 0x%x\n",
                     pkt->getAddr());

        assert(pkt->isRead());
        assert(!pkt->isError());

        // Fill the buffer. We assume that all packets arrive in order.
        pktBuffer.push(pkt);
    }
}

void
BaseDtu::tick()
{
    Addr messageSize = regFile.readReg(DtuRegister::SIZE);

    Addr bytesRead = readAddr - regFile.readReg(DtuRegister::SOURCE_ADDR);

    if (state == State::RECEIVING)
    {
        /*
         * TODO Limit buffer size. Send packats only when there is a free slot
         *      in the packet buffer (pktBuffer)
         */
        if (bytesRead < messageSize && isNocPortReady())
        {
            Addr addr = getDtuBaseAddr(regFile.readReg(DtuRegister::TARGET_COREID));

            addr += readAddr;

            Addr pktSize = messageSize - bytesRead;

            if (pktSize > maxPktSize)
                pktSize = maxPktSize;

            PacketPtr pkt = generateRequest(addr, pktSize, MemCmd::ReadReq);

            readAddr += pktSize;

            auto state = new SenderState(false);

            if (bytesRead + pktSize == messageSize)
                state->isLastRequest = true;

            pkt->pushSenderState(state);

            sendNocRequest(pkt);
        }

        if (!pktBuffer.empty() && isSpmPortReady())
        {
            // the buffer contains responses from NoC read requests
            PacketPtr nocPkt = pktBuffer.front();
            pktBuffer.pop();

            Addr pktSize = nocPkt->getSize();

            PacketPtr pkt = generateRequest(writeAddr, pktSize, MemCmd::WriteReq);

            writeAddr += pktSize;

            auto senderState = nocPkt->popSenderState();

            pkt->pushSenderState(senderState);

            memcpy(pkt->getPtr<uint8_t>(), nocPkt->getPtr<uint8_t>(), nocPkt->getSize());

            sendSpmRequest(pkt);

            delete nocPkt->req;
            delete nocPkt;
        }
    }
    else if (state == State::TRANSMITTING)
    {
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

            if (bytesRead + pktSize == messageSize)
                state->isLastRequest = true;

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

            auto senderState = spmPkt->popSenderState();

            PacketPtr pkt = generateRequest(paddr, pktSize, MemCmd::WriteReq);

            pkt->pushSenderState(senderState);

            memcpy(pkt->getPtr<uint8_t>(), spmPkt->getPtr<uint8_t>(), spmPkt->getSize());

            sendNocRequest(pkt);

            delete spmPkt->req;
            delete spmPkt;
        }
    }

    schedule(tickEvent, nextCycle());
}
