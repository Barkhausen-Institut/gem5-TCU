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

#ifndef __CPU_DTUTEST_DTUTEST_HH__
#define __CPU_DTUTEST_DTUTEST_HH__

#include "params/DtuTest.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuTest : public MemObject
{
  public:
    DtuTest(const DtuTestParams *p);

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

  protected:

    /// main simulation loop
    void tick();

    EventWrapper<DtuTest, &DtuTest::tick> tickEvent;

    class CpuPort : public MasterPort
    {
      private:
        DtuTest& dtutest;
      public:
        CpuPort(const std::string& _name, DtuTest* _dtutest)
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
        INIT,
        SETUP_TRANSMIT_EP,
        SETUP_RECEIVE_EP,
        START_TRANSMISSION,
        WAIT,
        VALIDATE,
    };

    State state;

    /// Request id for all generated traffic
    MasterID masterId;

    unsigned int id;

    const bool atomic;

    // XXX used to count ticks and do different tasks depending on current value
    int counter = 0;

    /// Stores the Packet for later retry
    PacketPtr retryPkt;

    PacketPtr createDtuRegisterPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    bool sendPkt(PacketPtr pkt);

    void completeRequest(PacketPtr pkt);

    void recvRetry();

    static Addr getRegAddr(DtuReg reg);

    static Addr getRegAddr(CmdReg reg);

    static Addr getRegAddr(EpReg reg, unsigned epid);
};

#endif // __CPU_DTUTEST_DTUTEST_HH__
