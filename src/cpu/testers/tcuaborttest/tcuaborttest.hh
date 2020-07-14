/*
 * Copyright (c) 2016, Nils Asmussen
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

#ifndef __CPU_TCUABORTTEST_TCUABORTTEST_HH__
#define __CPU_TCUABORTTEST_TCUABORTTEST_HH__

#include "params/TcuAbortTest.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/reg_file.hh"
#include "sim/system.hh"

class TcuAbortTest : public ClockedObject
{
  public:
    TcuAbortTest(const TcuAbortTestParams *p);

    Port& getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  protected:

    /// main simulation loop
    void tick();

    EventWrapper<TcuAbortTest, &TcuAbortTest::tick> tickEvent;

    class CpuPort : public MasterPort
    {
      private:
        TcuAbortTest& tcutest;
      public:
        CpuPort(const std::string& _name, TcuAbortTest* _tcutest)
            : MasterPort(_name, _tcutest), tcutest(*_tcutest)
        { }
      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    CpuPort port;

    enum class State
    {
        IDLE,
        INIT_MEM,
        INIT_MEM_EP,
        INIT_SEND_EP,
        INIT_RECV_EP,
        TESTS,
        STOP,
    };

    enum class SubState
    {
        START,
        ABORT,
        WAIT_ABORT,
        WAIT_CMD,
    };

    State state;
    SubState substate;
    uint testNo;
    uint delay;
    uint abortType;
    uint abortTypes;

    Tick abortStart;

    System *system;

    Addr reg_base;

    /// Request id for all generated traffic
    MasterID masterId;

    unsigned int id;

    const bool atomic;

    /// Stores the Packet for later retry
    PacketPtr retryPkt;

    PacketPtr createPacket(Addr paddr, size_t size, MemCmd cmd);

    PacketPtr createTcuRegisterPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    PacketPtr createCommandPkt(CmdCommand::Bits cmd,
                               CmdData::Bits data,
                               Addr arg1 = 0);

    bool sendPkt(PacketPtr pkt);

    void completeRequest(PacketPtr pkt);

    void recvRetry();

    void finishTest(bool success);

    static Addr getRegAddr(PrivReg reg);

    static Addr getRegAddr(UnprivReg reg);

    static Addr getRegAddr(unsigned reg, unsigned epid);
};

#endif // __CPU_TCUABORTTEST_TCUABORTTEST_HH__
