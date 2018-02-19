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

#ifndef __CPU_DTU_ACCEL_HH__
#define __CPU_DTU_ACCEL_HH__

#include "params/DtuAccel.hh"
#include "cpu/dtu-accel-hash/algorithm.hh"
#include "mem/dtu/connector/base.hh"
#include "mem/dtu/regfile.hh"
#include "mem/dtu/dtu.hh"
#include "sim/system.hh"

class DtuAccel : public MemObject
{
  public:

    static const unsigned EP_SYSS;
    static const unsigned EP_SYSR;

    static const Addr MSG_ADDR;

    static const Addr RCTMUX_YIELD;
    static const Addr RCTMUX_FLAGS;

    DtuAccel(const DtuAccelParams *p);

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    void setConnector(BaseConnector *con)
    {
        connector = con;
    }

    virtual void wakeup() {
    }

    virtual void interrupt() = 0;

    virtual void reset() = 0;

    virtual void signalFinished() {};

    /// main simulation loop
    virtual void tick() = 0;

    virtual void completeRequest(PacketPtr pkt) = 0;

    virtual Addr bufferAddr() const = 0;

    virtual int contextEp() const = 0;
    virtual size_t stateSize() const = 0;
    virtual size_t contextSize() const = 0;
    virtual void *context() = 0;
    virtual void setSwitched() = 0;

    class CpuPort : public MasterPort
    {
      private:
        DtuAccel& dtuaccel;
      public:
        CpuPort(const std::string& _name, DtuAccel* _dtuaccel)
            : MasterPort(_name, _dtuaccel), dtuaccel(*_dtuaccel)
        { }
      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    enum RCTMuxCtrl
    {
        NONE        = 0,
        STORE       = 1 << 0, // store operation required
        RESTORE     = 1 << 1, // restore operation required
        WAITING     = 1 << 2, // set by the kernel if a signal is required
        SIGNAL      = 1 << 3, // used to signal completion to the kernel
    };

    PacketPtr createPacket(Addr paddr, size_t size, MemCmd cmd);

    PacketPtr createPacket(Addr paddr, const void *data, size_t size, MemCmd cmd);

    PacketPtr createDtuRegPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    PacketPtr createDtuCmdPkt(Dtu::Command::Opcode cmd, unsigned epid,
                              uint64_t data, uint64_t size, uint64_t arg);

    void freePacket(PacketPtr pkt);

    bool sendPkt(PacketPtr pkt);

    void recvRetry();

    static Addr getRegAddr(DtuReg reg);

    static Addr getRegAddr(ReqReg reg);

    static Addr getRegAddr(CmdReg reg);

    static Addr getRegAddr(unsigned reg, unsigned epid);

    System *system;

    EventWrapper<DtuAccel, &DtuAccel::tick> tickEvent;

    bool haveVM;
    Addr chunkSize;
    size_t maxDataSize;

  private:

    CpuPort port;

    /// Request id for all generated traffic
    MasterID masterId;

    unsigned int id;

    const bool atomic;

    Addr reg_base;

    /// Stores the Packet for later retry
    PacketPtr retryPkt;

    BaseConnector *connector;
};

#endif // __CPU_DTU_ACCEL_HH__
