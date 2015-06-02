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

#include "cpu/testers/dtutest/dtutest.hh"
#include "debug/DtuTest.hh"
#include "debug/DtuTestAccess.hh"
#include "mem/dtu/regfile.hh"

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
    id = p->id;
    TESTER_DTU++;

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

            if (state == State::WAIT)
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

                if (addr < 128)
                {
                    if (*data != static_cast<uint8_t>(addr))
                        panic("Found a Memory error at %#x!\n", addr);
                }
                else if (addr == 128)
                {
                    if (*data != (id + TESTER_DTU - 1) % TESTER_DTU)
                        panic("Found a Memory error at %#x!\n", addr);
                }
                else if (addr == 129)
                {
                    if (*data != 0)
                        panic("Found a Memory error at %#x!\n", addr);
                }
                else if (addr == 130)
                {
                    if (*data != 128)
                        panic("Found a Memory error at %#x!\n", addr);
                }
                else if (addr == 131)
                {
                    if (*data != 0)
                        panic("Found a Memory error at %#x!\n", addr);
                }
                else
                {
                    if (*data != static_cast<uint8_t>(addr - 128 - 4))
                        panic("Found a Memory error at %#x!\n", addr);
                }
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
DtuTest::createDtuRegisterPkt(Addr reg,
                              uint32_t value,
                              MemCmd cmd = MemCmd::WriteReq)
{
    // TODO parameterize
    Addr paddr = 0x10000000 + reg;

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

    Addr regAddr;

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

        if (counter == 128)
        {
            DPRINTF(DtuTest, "Done initializing the Scratchpad\n");
            counter = 0;
            state = State::SETUP_TRANSMIT_EP;
        }

        break;
    }
    case State::SETUP_TRANSMIT_EP:
    {
        switch (counter)
        {
        case 0:
            DPRINTF(DtuTest, "Setup Ep 0 to transmit 128 Bytes from local "
                             "Scratchpad at address 0x0 to Ep 1 at core %u\n",
                             TESTER_DTU, (id + 1) % TESTER_DTU);
            regAddr = RegFile::getRegAddr(EpReg::MESSAGE_ADDR, 0);
            pkt = createDtuRegisterPkt(regAddr, 0);
            break;
        case 1:
            regAddr = RegFile::getRegAddr(EpReg::MESSAGE_SIZE, 0);
            pkt = createDtuRegisterPkt(regAddr, 128);
            break;
        case 2:
            regAddr = RegFile::getRegAddr(EpReg::TARGET_EPID, 0);
            pkt = createDtuRegisterPkt(regAddr, 1);
            break;
        case 3:
            regAddr = RegFile::getRegAddr(EpReg::TARGET_COREID, 0);
            pkt = createDtuRegisterPkt(regAddr, (id + 1) % TESTER_DTU);
            break;
        case 4:
            regAddr = RegFile::getRegAddr(EpReg::CONFIG, 0);
            pkt = createDtuRegisterPkt(regAddr, 1);
            break;
        default:
            counter = -1;
            DPRINTF(DtuTest, "EP 0 setup done.\n");
            state = State::SETUP_RECEIVE_EP;
            break;
        }

        counter++;

        break;
    }
    case State::SETUP_RECEIVE_EP:
    {
        switch (counter)
        {
        case 0:
            DPRINTF(DtuTest, "Setup EP 1 to receive messages\n");
            regAddr = RegFile::getRegAddr(EpReg::CONFIG, 1);
            pkt = createDtuRegisterPkt(regAddr, 0);
            break;
        case 1:
            regAddr = RegFile::getRegAddr(EpReg::BUFFER_ADDR, 1);
            pkt = createDtuRegisterPkt(regAddr, 128);
            break;
        case 2:
            regAddr = RegFile::getRegAddr(EpReg::BUFFER_SIZE, 1);
            pkt = createDtuRegisterPkt(regAddr, 2048);
            break;
        case 3:
            regAddr = RegFile::getRegAddr(EpReg::BUFFER_READ_PTR, 1);
            pkt = createDtuRegisterPkt(regAddr, 128);
            break;
        case 4:
            regAddr = RegFile::getRegAddr(EpReg::BUFFER_WRITE_PTR, 1);
            pkt = createDtuRegisterPkt(regAddr, 128);
            break;
        default:
            counter = -1;
            DPRINTF(DtuTest, "EP 1 setup done.\n");
            state = State::START_TRANSMISSION;
            break;
        }

        counter++;

        break;
    }
    case State::START_TRANSMISSION:
    {
        DPRINTF(DtuTest, "Start Transmission\n");
        regAddr = RegFile::getRegAddr(DtuReg::COMMAND);
        pkt = createDtuRegisterPkt(regAddr, 0x100);

        state = State::WAIT;
        counter = 0;

        break;
    }
    case State::WAIT:
    {
        regAddr = RegFile::getRegAddr(DtuReg::STATUS);
        pkt = createDtuRegisterPkt(regAddr, 0, MemCmd::ReadReq);

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

        if (counter == 256+4)
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
