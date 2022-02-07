/*
 * Copyright (C) 2021 Nils Asmussen, Barkhausen Institut
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

#ifndef __DEV_TCU_SERIAL_INPUT_HH__
#define __DEV_TCU_SERIAL_INPUT_HH__

#include "base/pollevent.hh"
#include "mem/qport.hh"
#include "params/TcuSerialInput.hh"
#include "sim/clocked_object.hh"
#include "cmd_sm.hh"

#include <termios.h>

class TcuSerialInput : public ClockedObject, public CommandExecutor
{
  static const int EP_INPUT = 4;

  protected:
    class DataEvent : public PollEvent
    {
      protected:
        TcuSerialInput *ser;

      public:
        DataEvent(TcuSerialInput *s, int fd, int e);
        void process(int revent);
    };

    friend class DataEvent;
    DataEvent *dataEvent;

  private:
    class TcuMasterPort : public MasterPort
    {
      private:
        TcuSerialInput& serialInput;
        ReqPacketQueue reqPacketQueue;

      public:
        TcuMasterPort(const std::string& _name, TcuSerialInput* _serialInput)
            : MasterPort(_name, _serialInput),
              serialInput(*_serialInput),
              reqPacketQueue(*_serialInput, *this)
        {
        }

        void schedTimingReq(PacketPtr pkt, Tick when)
        {
            reqPacketQueue.schedSendTiming(pkt, when);
        }

        bool trySatisfyFunctional(PacketPtr pkt)
        {
            return reqPacketQueue.trySatisfyFunctional(pkt);
        }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override { reqPacketQueue.retry(); }
    };

    class TcuSlavePort : public QueuedResponsePort
    {
      private:
        TcuSerialInput& serialInput;
        RespPacketQueue respPacketQueue;

      public:
        TcuSlavePort(const std::string& _name, TcuSerialInput* _serialInput)
            : QueuedResponsePort(_name, _serialInput, respPacketQueue),
              serialInput(*_serialInput),
              respPacketQueue(*_serialInput, *this)
        {
        }

      protected:
        void recvFunctional(PacketPtr pkt) override;

        bool recvTimingReq(PacketPtr pkt) override;

        Tick recvAtomic(PacketPtr pkt) override;

        AddrRangeList getAddrRanges() const override;
    };

  public:
    void tick();

  protected:
    TcuMasterPort tcuMasterPort;
    TcuSlavePort tcuSlavePort;
    EventWrapper<TcuSerialInput, &TcuSerialInput::tick> tickEvent;
    TcuIf tcu;
    CommandSM cmdSM;
    struct termios old;
    char buffer[64];
    size_t pos;

    void data();

  public:
    typedef TcuSerialInputParams Params;
    TcuSerialInput(const Params &p);
    ~TcuSerialInput();

    void init() override;

    Port& getPort(const std::string& if_name,
                  PortID idx = InvalidPortID) override;

    std::string execName() override { return name(); }
    void commandFinished() override;
    void scheduleCommand(Cycles delay) override;
    void sendMemoryReq(PacketPtr pkt, Cycles delay) override;
};

#endif // __DEV_TCU_SERIAL_INPUT_HH__
