/*
 * Copyright (c) 2015, Christian Menard
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 */

#include "cpu/testers/dtutest/dtutest.hh"
#include "debug/DtuTest.hh"
#include "debug/DtuTestAccess.hh"

unsigned int TESTER_DTU = 0;

bool
DtuTest::CpuPort::recvTimingResp(PacketPtr pkt)
{
    dtutest.completeRequest(pkt);
    return true;
}

void
DtuTest::CpuPort::recvReqRetry()
{
    dtutest.recvRetry();
}

DtuTest::DtuTest(const DtuTestParams *p)
  : MemObject(p),
    tickEvent(this),
    port("port", this),
    masterId(p->system->getMasterId(name())),
    atomic(p->system->isAtomicMode()),
    retryPkt(nullptr)
{
    id = TESTER_DTU++;

    // Only every second dtu performs transactions
    if (id % 2)
        state = State::IDLE;
    else
        state = State::INIT;

    // kick things into action
    schedule(tickEvent, curTick());
}

BaseMasterPort &
DtuTest::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return MemObject::getMasterPort(if_name, idx);
}

bool
DtuTest::sendPkt(PacketPtr pkt)
{
    DPRINTF(DtuTestAccess, "Send %s %s request at address 0x%x\n",
                           atomic ? "atomic" : "timed",
                           pkt->isWrite() ? "write" : "read",
                           pkt->getAddr());

    if (atomic)
    {
        port.sendAtomic(pkt);
        completeRequest(pkt);
    }
    else
    {
        bool retry = !port.sendTimingReq(pkt);

        if (retry)
        {
            retryPkt = pkt;
            return false;
        }
    }

    return true;
}

void
DtuTest::completeRequest(PacketPtr pkt)
{
    Request* req = pkt->req;

    DPRINTF(DtuTestAccess, "Completing %s at address %x %s\n",
                           pkt->isWrite() ? "write" : "read",
                           req->getPaddr(),
                           pkt->isError() ? "error" : "success");

    const uint8_t *pkt_data = pkt->getConstPtr<uint8_t>();

    if (pkt->isError())
    {
        warn("%s access failed at %#x\n",
             pkt->isWrite() ? "Write" : "Read", req->getPaddr());
    }
    else
    {
        if (pkt->isRead())
        {
            DPRINTF(DtuTestAccess, "%s: read of %x @ cycle %d returns %x\n",
                                   name(),
                                   req->getPaddr(),
                                   curTick(),
                                   pkt_data[0]);

            if (state == State::WAIT_FOR_DTU_TRANSMIT)
            {
                if (pkt_data[0] == 0)
                {
                    state = State::SETUP_DTU_RECEIVE;
                    counter = 0;
                }
            }
            else if (state == State::WAIT_FOR_DTU_RECEIVE)
            {
                if (pkt_data[0] == 0)
                {
                    state = State::VALIDATE;
                    counter = 0;
                }
            }
            else if (state == State::VALIDATE)
            {
                Addr addr = pkt->getAddr();
                uint8_t* data = pkt->getPtr<uint8_t>();

                if (*data != static_cast<uint8_t>(addr))
                    panic("Found a Memory error!");
            }
        }
    }

    delete pkt->req;

    // the packet will delete the data
    delete pkt;

    // kick things into action again
    schedule(tickEvent, clockEdge(Cycles(1)));
}

void
DtuTest::recvRetry()
{
    assert(retryPkt);
    if (port.sendTimingReq(retryPkt))
    {
        DPRINTF(DtuTestAccess, "Proceeding after successful retry\n");

        retryPkt = nullptr;
    }
}

PacketPtr
DtuTest::createDtuRegisterPkt(DtuRegister reg, uint32_t value, MemCmd cmd = MemCmd::WriteReq)
{
    // TODO parameterize
    Addr paddr = 0x10000000 + static_cast<Addr>(reg) * 4;

    Request::Flags flags;

    auto req = new Request(paddr, 4, flags, masterId);
    req->setThreadContext(id, 0);

    auto pkt = new Packet(req, cmd);
    auto pkt_data = new uint32_t;
    *pkt_data = value;
    pkt->dataDynamic(pkt_data);

    return pkt;
}

void
DtuTest::tick()
{
    PacketPtr pkt = nullptr;

    switch (state)
    {
    case State::IDLE:
        break;
    case State::INIT:
    {
        if (counter == 0)
            DPRINTF(DtuTest, "Initialize the first 1024 byts of Scratchpad-Memory\n");

        Addr paddr = counter;
        Request::Flags flags;

        auto req = new Request(paddr, 1, flags, masterId);
        req->setThreadContext(id, 0);

        pkt = new Packet(req, MemCmd::WriteReq);
        auto pkt_data = new uint8_t[1];
        pkt->dataDynamic(pkt_data);
        pkt_data[0] = static_cast<uint8_t>(counter);

        counter++;

        if (counter == 1024)
        {
            DPRINTF(DtuTest, "Done initializing the Scratchpad\n");
            counter = 0;
            state = State::SETUP_DTU_TRANSMIT;
        }

        break;
    }
    case State::SETUP_DTU_TRANSMIT:
    {
        switch (counter)
        {
        case 0:
            DPRINTF(DtuTest, "Setup the DTU to transmit 1024 Bytes form local "
                             "Scratchpad at address 0x0 to core %u's Scratchpad "
                             "at address 0x400\n", id + 1);
            pkt = createDtuRegisterPkt(DtuRegister::SOURCE_ADDR, 0);
            break;
        case 1:
            pkt = createDtuRegisterPkt(DtuRegister::SIZE, 1024);
            break;
        case 2:
            pkt = createDtuRegisterPkt(DtuRegister::TARGET_ADDR, 1024);
            break;
        case 3:
            pkt = createDtuRegisterPkt(DtuRegister::TARGET_COREID, id + 1);
            break;
        case 4:
            pkt = createDtuRegisterPkt(DtuRegister::COMMAND, BaseDtu::TRANSMIT_CMD);
            break;
        default:
            counter = 0;
            DPRINTF(DtuTest, "DTU setup done. Wait until DTU finishes the transaction\n");
            state = State::WAIT_FOR_DTU_TRANSMIT;
            break;
        }

        counter++;

        break;
    }
    case State::WAIT_FOR_DTU_TRANSMIT:
    case State::WAIT_FOR_DTU_RECEIVE:

        pkt = createDtuRegisterPkt(DtuRegister::STATUS, 0, MemCmd::ReadReq);

        break;

    case State::SETUP_DTU_RECEIVE:
    {
        switch (counter)
        {
        case 0:
            DPRINTF(DtuTest, "DTU finished the transaction. Setup the DTU to "
                             "read 1024 bytes from core %u's scratchpad at "
                             "address 0x400 to local scratchpad at address "
                             "0x400\n", id+1);
            pkt = createDtuRegisterPkt(DtuRegister::SOURCE_ADDR, 1024);
            break;
        case 1:
            pkt = createDtuRegisterPkt(DtuRegister::SIZE, 1024);
            break;
        case 2:
            pkt = createDtuRegisterPkt(DtuRegister::TARGET_ADDR, 1024);
            break;
        case 3:
            pkt = createDtuRegisterPkt(DtuRegister::TARGET_COREID, id + 1);
            break;
        case 4:
            pkt = createDtuRegisterPkt(DtuRegister::COMMAND, BaseDtu::RECEIVE_CMD);
            break;
        default:
            counter = 0;
            DPRINTF(DtuTest, "DTU setup done. Wait until DTU finishes the transaction\n");
            state = State::WAIT_FOR_DTU_RECEIVE;
            break;
        }

        counter++;

        break;
    }
    case State::VALIDATE:
    {
        if (counter == 0)
            DPRINTF(DtuTest, "DTU finished the transaction. Validate the "
                             "Scratchpad content.\n");

        Addr paddr = counter;
        Request::Flags flags;

        auto req = new Request(paddr, 1, flags, masterId);
        req->setThreadContext(id, 0);

        pkt = new Packet(req, MemCmd::ReadReq);
        auto pkt_data = new uint8_t[1];
        pkt->dataDynamic(pkt_data);

        counter++;

        if (counter == 2024)
        {
            DPRINTF(DtuTest, "Successfully verified Scratchpad data.\n");
            counter = 0;
            state = State::IDLE;
        }

        break;
    }
    }

    // Schedule next tick if the packet was successfully send.
    // Otherwise block until a retry is received.

    if (pkt == nullptr)
        schedule(tickEvent, clockEdge(Cycles(1)));
    else
        sendPkt(pkt);
}

DtuTest*
DtuTestParams::create()
{
    return new DtuTest(this);
}
