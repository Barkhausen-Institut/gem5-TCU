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

#ifndef __DEV_KECACC_HH__
#define __DEV_KECACC_HH__

#include "dev/io_device.hh"
#include "dev/kecacc/kecacc-xkcp.hh"
#include "params/KecAcc.hh"

class KecAcc : public BasicPioDevice
{
  public:
    PARAMS(KecAcc);
    explicit KecAcc(const Params &p);
    ~KecAcc() override;

    Port &getPort(const std::string &if_name, PortID idx) override;

  protected:
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  private:
    class CPUPort : public RequestPort
    {
      public:
        CPUPort(const std::string &name, KecAcc &acc);
        ~CPUPort() override;
        void sendTimingReqRetry(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;

      private:
        KecAcc &acc;
        PacketPtr retryPkt;
    };

    void startAbsorb();
    void startSqueeze();
    Cycles absorb(PacketPtr pkt);
    Cycles squeeze(PacketPtr pkt);
    Cycles pad();
    void completeIO(PacketPtr pkt);
    void finishCommand();

    PacketPtr createMemReq(MemCmd memCmd, Addr addr,
                           void *ptr, unsigned size) const;
    PacketPtr createSpongeMemReq(MemCmd memCmd) const;
    void sendPkt(PacketPtr pkt, Cycles delay = Cycles(0));
    void sendDelayedPkt();

    CPUPort port;
    RequestorID requestorId;
    PacketPtr delayedPkt;
    EventWrapper<KecAcc, &KecAcc::sendDelayedPkt> sendDelayedPktEvent;
    EventWrapper<KecAcc, &KecAcc::finishCommand> finishCmdEvent;

    uint64_t cmd;
    KecAccXKCP xkcp;
    std::unique_ptr<uint8_t[]> buffer;
    Cycles start;
};

#endif // __DEV_KECACC_HH__
