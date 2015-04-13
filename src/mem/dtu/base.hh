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

#ifndef __MEM_DTU_BASE_HH__
#define __MEM_DTU_BASE_HH__

#include "mem/dtu/regfile.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"
#include "params/Dtu.hh"

#include <queue>

/**
 * This class implements the actual DTU functionality.
 */
class BaseDtu : public MemObject
{
  public:

    static constexpr DtuReg RECEIVE_CMD = 1;
    static constexpr DtuReg TRANSMIT_CMD = 2;
    static constexpr DtuReg IDLE_STATUS = 0;
    static constexpr DtuReg BUSY_STATUS = 1;

  protected:

    enum class State
    {
        IDLE,
        RECEIVING,
        TRANSMITTING,
    };

    RegFile regFile;

    const Addr cpuBaseAddr;

    Addr nocBaseAddr;

    const unsigned nocAddrBits;

    const unsigned maxPktSize;

    MasterID masterId;

    State state;

    bool waitingForSpmRetry;

    bool waitingForNocRetry;

    void tick();

    EventWrapper<BaseDtu, &BaseDtu::tick> tickEvent;

    Addr readAddr;

    Addr writeAddr;

    std::queue<uint8_t> buffer;

    PacketPtr generateRequest(Addr paddr, Addr size, MemCmd cmd);

    Addr getDtuBaseAddr(unsigned coreId) const;

    void startTransaction(DtuReg cmd);

    void sendNextSpmReadRequest();

    virtual bool sendSpmRequest(PacketPtr pkt) = 0;

    virtual bool sendNocRequest(PacketPtr pkt) = 0;

    void completeSpmRequest(PacketPtr pkt);

    void completeNocRequest(PacketPtr pkt);

  public:

    BaseDtu(const BaseDtuParams* p);

    Tick handleCpuRequest(PacketPtr pkt);
};

#endif // __MEM_DTU_BASE_HH__
