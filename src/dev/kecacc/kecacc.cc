/*
 * Copyright (c) 2021 Stephan Gerhold
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "dev/kecacc/kecacc.hh"

#include "base/bitunion.hh"
#include "base/trace.hh"
#include "debug/KecAcc.hh"
#include "sim/system.hh"

namespace {
enum
{
    // Accelerator is idle (has completed previous command)
    CMD_IDLE,
    // Initialize accelerator with specified hash type
    CMD_INIT,
    // Load accelerator state from specified memory address
    CMD_LOAD,
    // Save accelerator state to specified memory address
    CMD_SAVE,
    // Absorb bytes from specified memory address
    CMD_ABSORB,
    // Absorb last few bytes from specified memory address and apply padding
    CMD_ABSORB_LAST,
    // Squeeze bytes to specified memory address
    CMD_SQUEEZE,
};

BitUnion64(CmdReg)
    BitfieldRO<3, 0> cmd;
    SubBitUnion(init, 63, 4)
        BitfieldRO<7, 4> hash_type;
    EndSubBitUnion(init)
    SubBitUnion(state, 63, 4)
        BitfieldRO<33, 4> addr;
    EndSubBitUnion(state)
    SubBitUnion(sponge, 63, 4)
        BitfieldRO<33, 4> addr;
        BitfieldRO<63, 34> bytes;
    EndSubBitUnion(sponge)
EndBitUnion(CmdReg)
}

KecAcc::KecAcc(const Params &p)
    : BasicPioDevice(p, sizeof(uint64_t)), port("port", *this),
      requestorId(p.system->getRequestorId(this)), delayedPkt(nullptr),
      sendDelayedPktEvent(this), finishCmdEvent(this),
      cmd(0), xkcp(), buffer(new uint8_t[p.buf_size + alignof(uint64_t)]),
      start(0)
{
}

KecAcc::~KecAcc()
{
    delete delayedPkt;
}

Port &
KecAcc::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    return PioDevice::getPort(if_name, idx);
}

Tick
KecAcc::read(PacketPtr pkt)
{
    auto reg_addr = pkt->getAddr() - pioAddr;
    fatal_if(reg_addr, "Unsupported register: %#lx\n", reg_addr);

    uint64_t val = cmd;
    //DPRINTF(KecAcc, "Read register %#lx = %#lx\n", reg_addr, val);

    pkt->setUintX(val, ByteOrder::little);
    pkt->makeResponse();
    return pioDelay;
}

Tick
KecAcc::write(PacketPtr pkt)
{
    auto reg_addr = pkt->getAddr() - pioAddr;
    fatal_if(reg_addr, "Unsupported register: %#lx\n", reg_addr);

    auto val = pkt->getUintX(ByteOrder::little);
    DPRINTF(KecAcc, "Write register %#lx = %#lx\n", reg_addr, val);

    fatal_if(cmd, "Cannot write while accelerator is busy\n");
    cmd = val;
    start = curCycle();

    CmdReg reg(val);

    switch (reg.cmd) {
      case CMD_INIT:
      case CMD_LOAD:
        break;
      default:
        fatal_if(!xkcp.rate_bytes(), "Accelerator not initialized\n");
    }

    switch (reg.cmd) {
      case CMD_INIT:
        fatal_if(!xkcp.init(reg.init.hash_type),
                 "Unsupported hash type: %d\n", reg.init.hash_type);
        schedule(finishCmdEvent, clockEdge(params().init_cycles));
        break;
      case CMD_LOAD:
        sendPkt(createMemReq(MemCmd::ReadReq, reg.state.addr, xkcp.state,
                             sizeof(xkcp.state)));
        break;
      case CMD_SAVE:
        sendPkt(createMemReq(MemCmd::WriteReq, reg.state.addr, xkcp.state,
                             sizeof(xkcp.state)), params().save_cycles);
        break;
      case CMD_ABSORB:
        if (reg.sponge.bytes == 0) {
            schedule(finishCmdEvent, nextCycle());
            break;
        }
        startAbsorb();
        break;
      case CMD_ABSORB_LAST:
        if (reg.sponge.bytes == 0) {
            schedule(finishCmdEvent, clockEdge(pad()));
            break;
        }
        startAbsorb();
        break;
      case CMD_SQUEEZE:
        if (reg.sponge.bytes == 0) {
            schedule(finishCmdEvent, nextCycle());
            break;
        }
        startSqueeze();
        break;
      default:
        fatal("Unsupported command: %d\n", reg.cmd);
    }

    pkt->makeResponse();
    return pioDelay;
}

PacketPtr
KecAcc::createMemReq(MemCmd memCmd, Addr addr, void *ptr, unsigned size) const
{
    Request::Flags flags;
    auto req = std::make_shared<Request>(addr, size, flags, requestorId);
    auto pkt = new Packet(req, memCmd);
    pkt->dataStatic(ptr);

    // Calculate memory access time with selected mem width of accelerator
    Cycles delay(divCeil(size, params().mem_width));
    pkt->payloadDelay = cyclesToTicks(delay);
    return pkt;
}

PacketPtr
KecAcc::createSpongeMemReq(MemCmd memCmd) const
{
    CmdReg reg = cmd;

    // Since the entire buffer is loaded at once it must fit into the
    // configured buffer size. It can be increased in the configuration.
    fatal_if(reg.sponge.bytes > params().buf_size,
             "Buffer too small for requested bytes\n");

    // Adjust buffer start according to backend alignment offset
    auto buffer_start = buffer.get() + xkcp.alignment_offset();
    auto pkt = createMemReq(memCmd, reg.sponge.addr,
                            buffer_start, reg.sponge.bytes);

    // Calculate cycles per block which include an additional pipeline overhead
    unsigned int rate = xkcp.rate_bytes();
    Cycles cpb(divCeil(rate, params().mem_width) + params().pipeline_cycles);

    // Adjust memory access time according to number of blocks in the buffer
    auto blocks = pkt->getSize() / rate;
    auto remaining = pkt->getSize() % rate;
    Cycles delay(blocks * cpb + divCeil(remaining, params().mem_width));
    pkt->payloadDelay = cyclesToTicks(delay);
    return pkt;
}

void
KecAcc::sendPkt(PacketPtr pkt, Cycles delay)
{
    DPRINTF(KecAcc,
            "sendPkt: %s (addr: %#lx, size: %u, cycles: %llu, delay: %llu)\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize(),
            ticksToCycles(pkt->payloadDelay), delay);

    if (delay) {
        delayedPkt = pkt;
        schedule(&sendDelayedPktEvent, clockEdge(delay));
    } else {
        port.sendTimingReqRetry(pkt);
    }
}

void
KecAcc::sendDelayedPkt()
{
    assert(delayedPkt);
    port.sendTimingReqRetry(delayedPkt);
    delayedPkt = nullptr;
}

void
KecAcc::startAbsorb()
{
    CmdReg reg = cmd;
    fatal_if(reg.sponge.bytes > params().buf_size,
             "Buffer too small for requested bytes\n");
    sendPkt(createSpongeMemReq(MemCmd::ReadReq));
}

void
KecAcc::startSqueeze()
{
    CmdReg reg = cmd;
    fatal_if(reg.sponge.bytes > params().buf_size,
             "Buffer too small for requested bytes\n");
    auto pkt = createSpongeMemReq(MemCmd::WriteReq);
    sendPkt(pkt, squeeze(pkt));
}

Cycles
KecAcc::absorb(PacketPtr pkt)
{
    DPRINTF(KecAcc, "absorb(%u)\n", pkt->getSize());
    assert(pkt->isRead());
    auto nperm = xkcp.absorb(pkt->getConstPtr<uint8_t>(), pkt->getSize());
    return Cycles(std::max(nperm * params().permute_cycles, 1UL));
}

Cycles
KecAcc::squeeze(PacketPtr pkt)
{
    DPRINTF(KecAcc, "squeeze(%u)\n", pkt->getSize());
    assert(pkt->isWrite());
    auto nperm = xkcp.squeeze(pkt->getPtr<uint8_t>(), pkt->getSize());
    return Cycles(std::max(nperm * params().permute_cycles, 1UL));
}

Cycles
KecAcc::pad()
{
    DPRINTF(KecAcc, "pad\n");
    xkcp.pad();
    return params().pad_cycles;
}

void
KecAcc::completeIO(PacketPtr pkt)
{
    CmdReg reg = cmd;
    Cycles c(0);
    DPRINTF(KecAcc, "completeIO(%d)\n", reg.cmd);

    switch (reg.cmd) {
      case CMD_LOAD:
        assert(pkt->isRead());
        c = params().load_cycles;
        break;
      case CMD_ABSORB:
        c = absorb(pkt);
        break;
      case CMD_ABSORB_LAST:
        c = absorb(pkt);
        c += pad();
        break;
      case CMD_SAVE:
      case CMD_SQUEEZE:
        assert(pkt->isWrite());
        break;
      default:
        panic("Unexpected I/O completion for cmd: %d\n", reg.cmd);
    }

    delete pkt;
    schedule(finishCmdEvent, clockEdge(c));
}

void
KecAcc::finishCommand()
{
    DPRINTF(KecAcc, "finishCommand(%#lx) in %llu cycles\n",
            cmd, curCycle() - start);
    cmd = CMD_IDLE;
}

KecAcc::CPUPort::CPUPort(const std::string &name, KecAcc &acc)
    : RequestPort(name, &acc), acc(acc), retryPkt(nullptr)
{
}

KecAcc::CPUPort::~CPUPort()
{
    delete retryPkt;
}

void
KecAcc::CPUPort::sendTimingReqRetry(PacketPtr pkt)
{
    assert(!retryPkt);
    if (!sendTimingReq(pkt))
        retryPkt = pkt;
}

bool
KecAcc::CPUPort::recvTimingResp(PacketPtr pkt)
{
    acc.completeIO(pkt);
    return true;
}

void
KecAcc::CPUPort::recvReqRetry()
{
    assert(retryPkt);
    if (sendTimingReq(retryPkt))
        retryPkt = nullptr;
}
