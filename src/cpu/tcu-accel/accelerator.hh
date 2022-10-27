/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2021 Nils Asmussen, Barkhausen Institut
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

#ifndef __CPU_TCU_ACCEL_HH__
#define __CPU_TCU_ACCEL_HH__

#include "params/TcuAccel.hh"
#include "mem/tcu/connector/base.hh"
#include "mem/tcu/reg_file.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/tcuif.hh"
#include "sim/system.hh"

namespace gem5
{
namespace tcu
{

class TcuAccel : public ClockedObject
{
  public:

    static const unsigned EP_SYSS;
    static const unsigned EP_SYSR;

    TcuAccel(const TcuAccelParams &p);

    Port& getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void setConnector(BaseConnector *con)
    {
        connector = con;
    }

    TcuIf &tcuif() {
      return tcu;
    }

    virtual void wakeup() {
    }

    virtual void interrupt() = 0;

    virtual void reset() = 0;

    /// main simulation loop
    virtual void tick() = 0;

    virtual void completeRequest(PacketPtr pkt) = 0;

    virtual Addr sendMsgAddr() const = 0;
    virtual Addr bufferAddr() const = 0;
    virtual void setSwitched() = 0;

    class CpuPort : public MasterPort
    {
      private:
        TcuAccel& tcuaccel;
      public:
        CpuPort(const std::string& _name, TcuAccel* _tcuaccel)
            : MasterPort(_name, _tcuaccel), tcuaccel(*_tcuaccel)
        { }
      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    enum PEMuxCtrl
    {
        NONE        = 0,
        RESTORE     = 1 << 0, // restore operation required
        WAITING     = 1 << 1, // set by the kernel if a signal is required
        SIGNAL      = 1 << 2, // used to signal completion to the kernel
    };

    bool sendPkt(PacketPtr pkt);

    void recvRetry();

    System *system;

    EventWrapper<TcuAccel, &TcuAccel::tick> tickEvent;

    bool haveVM;
    Addr chunkSize;
    size_t maxDataSize;

    const Addr offset;

  private:

    CpuPort port;

    TcuIf tcu;

    unsigned int id;

    const bool atomic;

    /// Stores the Packet for later retry
    PacketPtr retryPkt;

    BaseConnector *connector;
};

}
}

#endif // __CPU_TCU_ACCEL_HH__
