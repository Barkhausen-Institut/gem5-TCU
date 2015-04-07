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

#ifndef __MEM_DTU_DTU_HH__
#define __MEM_DTU_DTU_HH__

#include "mem/dtu/base.hh"
#include "mem/mem_object.hh"
#include "mem/tport.hh"
#include "params/Dtu.hh"

class Dtu : public MemObject, BaseDtu
{
  private:

    class DtuScratchpadPort : public MasterPort
    {
      public:

        DtuScratchpadPort(const std::string& _name, Dtu& _dtu)
          : MasterPort(_name, &_dtu)
        { }

      protected:

        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    class DtuCpuPort : public SimpleTimingPort
    {
      private:
        Dtu& dtu;

      public:

        DtuCpuPort(const std::string& _name, Dtu& _dtu)
          : SimpleTimingPort(_name, &_dtu), dtu(_dtu)
        { }

      protected:

        Tick recvAtomic(PacketPtr pkt) override;

        AddrRangeList getAddrRanges() const override;
    };

    class DtuMasterPort : public MasterPort
    {
      public:

        DtuMasterPort(const std::string& _name, Dtu& _dtu)
          : MasterPort(_name, &_dtu)
        { }

      protected:

        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    class DtuSlavePort : public SlavePort
    {
      private:
        Dtu& dtu;

      public:

        DtuSlavePort(const std::string& _name, Dtu& _dtu)
          : SlavePort(_name, &_dtu), dtu(_dtu)
        { }

      protected:

        Tick recvAtomic(PacketPtr pkt) override;

        void recvFunctional(PacketPtr pkt) override;

        bool recvTimingReq(PacketPtr pkt) override;

        void recvRespRetry() override;

        AddrRangeList getAddrRanges() const override;

    };

    Addr cpuBaseAddr;
    Addr size = 0x1000;

    Addr dtuAddr;
    unsigned dtuAddrBits;

    DtuCpuPort        cpu;
    DtuScratchpadPort scratchpad;
    DtuMasterPort     master;
    DtuSlavePort      slave;

    bool atomic;

    bool sendSpmPkt(PacketPtr pkt) override;

    bool sendNocPkt(PacketPtr pkt) override;

  public:

    Dtu(const DtuParams *p);

    void init() override;

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    BaseSlavePort& getSlavePort(const std::string &if_name,
                                PortID idx = InvalidPortID) override;
};

#endif // __MEM_DTU_DTU_HH__
