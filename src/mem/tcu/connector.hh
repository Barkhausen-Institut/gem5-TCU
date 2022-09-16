/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (C) 2020 Nils Asmussen, Barkhausen Institut
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

#ifndef __MEM_TCU_CONNECTOR_HH__
#define __MEM_TCU_CONNECTOR_HH__

#include "mem/tcu/connector/base.hh"
#include "mem/tcu/ep_file.hh"
#include "mem/tcu/error.hh"

class Tcu;

class TcuConnector
{
  public:

    TcuConnector(Tcu &_tcu, BaseConnector *_connector);

    const std::string name() const;

    void regStats();

    bool canSuspendCmds() const { return connector->canSuspendCmds(); }

    void reset() { connector->reset(); }

    void startWaitEP(const CmdCommand::Bits &cmd);

    void startWaitEPWithEP(EpFile::EpCache &eps, epid_t epid);

    bool startSleep(epid_t wakeupEp);

    void stopSleep();

    void wakeupCore(bool force, epid_t rep);

    void setIrq(BaseConnector::IRQ irq);

    void clearIrq(BaseConnector::IRQ irq);

    void fireTimer();

    void restartTimer(uint64_t nanos);

  private:

    Tcu &tcu;

    BaseConnector *connector;

    EpFile::EpCache sleepEPs;

    int wakeupEp;

    EventWrapper<TcuConnector, &TcuConnector::fireTimer> fireTimerEvent;

  public:

    Stats::Scalar irqInjects;

};

#endif // __MEM_TCU_CONNECTOR_HH__
