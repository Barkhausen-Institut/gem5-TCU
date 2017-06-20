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

#ifndef __CPU_DTUABORTTEST_DTUABORTTEST_HH__
#define __CPU_DTUABORTTEST_DTUABORTTEST_HH__

#include "params/DtuAbortTest.hh"
#include "mem/dtu/dtu.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuAbortTest : public MemObject
{
  public:
    DtuAbortTest(const DtuAbortTestParams *p);

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

  protected:

    /// main simulation loop
    void tick();

    EventWrapper<DtuAbortTest, &DtuAbortTest::tick> tickEvent;

    class CpuPort : public MasterPort
    {
      private:
        DtuAbortTest& dtutest;
      public:
        CpuPort(const std::string& _name, DtuAbortTest* _dtutest)
            : MasterPort(_name, _dtutest), dtutest(*_dtutest)
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
        WAIT,
        INVLPG_DATA,
        INVLPG_CMD,
        INVLPG_WAIT,
        REPAIR_REP,
        REPAIR_MSG,
    };

    State state;
    SubState substate;
    uint testNo;
    uint delay;

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

    PacketPtr createDtuRegisterPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    PacketPtr createCommandPkt(Dtu::Command::Opcode cmd,
                               unsigned ep,
                               Addr data,
                               Addr size,
                               Addr arg,
                               Addr off = 0);

    bool sendPkt(PacketPtr pkt);

    void completeRequest(PacketPtr pkt);

    void recvRetry();

    static Addr getRegAddr(DtuReg reg);

    static Addr getRegAddr(ReqReg reg);

    static Addr getRegAddr(CmdReg reg);

    static Addr getRegAddr(unsigned reg, unsigned epid);
};

#endif // __CPU_DTUABORTTEST_DTUABORTTEST_HH__
