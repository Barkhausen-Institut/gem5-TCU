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

#ifndef __MEM_DTU_BASE_DTU_HH__
#define __MEM_DTU_BASE_DTU_HH__

#include "mem/dtu/regfile.hh"
#include "mem/port.hh"
#include "params/Dtu.hh"
#include "sim/clocked_object.hh"

/**
 * This class implements the actual DTU functionality.
 */
class BaseDtu : public ClockedObject
{
  protected:

    static constexpr RegFile::IntReg RECEIVE_CMD = 1;
    static constexpr RegFile::IntReg TRANSMIT_CMD = 2;
    static constexpr RegFile::IntReg IDLE_STATUS = 0;
    static constexpr RegFile::IntReg BUSY_STATUS = 1;

    enum class State
    {
        IDLE,
        RECEIVING,
        TRANSMITTING,
    };

    MasterPort& spmPort;

    MasterPort& masterPort;

    RegFile regFile;

    Addr baseAddr;

    bool atomic;

    unsigned spmPktSize;

    unsigned nocPktSize;

    State state;

    void tick();

    EventWrapper<BaseDtu, &BaseDtu::tick> tickEvent;

    void startTransaction(RegFile::IntReg cmd);

  public:

    BaseDtu(const BaseDtuParams* p,
            MasterPort& _spmPort,
            MasterPort& _masterPort);

    Tick handleCpuRequest(PacketPtr pkt);

    bool sendSpmPkt(PacketPtr pkt);
};

#endif // __MEM_DTU_BASE_DTU_HH__
