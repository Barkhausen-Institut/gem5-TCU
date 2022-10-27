/*
 * Copyright (c) 2015, Christian Menard
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
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

#include "debug/TcuConnector.hh"
#include "mem/tcu/connector.hh"
#include "mem/tcu/tcu.hh"

namespace gem5
{
namespace tcu
{

TcuConnector::TcuConnector(Tcu &_tcu, BaseConnector *_connector)
    : tcu(_tcu),
      connector(_connector),
      sleepEPs(tcu.eps().newCache()),
      wakeupEp(0xFFFF),
      fireTimerEvent(*this)
{
    connector->setTcu(&tcu);
}

const std::string
TcuConnector::name() const
{
    return tcu.name();
}

void
TcuConnector::regStats()
{
    irqInjects
        .name(name() + ".irqInjects")
        .desc("Number of injected IRQs");
}

void
TcuConnector::startWaitEP(const CmdCommand::Bits &cmd)
{
    int ep = cmd.arg0 & 0xFFFF;

    if (ep == Tcu::INVALID_EP_ID)
    {
        if (tcu.regs().getAct(PrivReg::CUR_ACT).msgs > 0)
        {
            tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
            return;
        }

        if (!startSleep(ep))
            tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
    }
    else
    {
        sleepEPs.addEp(ep);
        sleepEPs.onFetched(std::bind(&TcuConnector::startWaitEPWithEP,
                                     this, std::placeholders::_1, ep));
    }
}

void
TcuConnector::startWaitEPWithEP(EpFile::EpCache &eps, epid_t epid)
{
    const Ep ep = eps.getEp(epid);
    if (ep.type() == EpType::RECEIVE && ep.recv.r2.unread != 0)
    {
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
        return;
    }

    if (!startSleep(epid))
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
}

bool
TcuConnector::startSleep(epid_t ep)
{
    if (connector->havePendingIrq())
        return false;

    wakeupEp = ep;
    DPRINTF(TcuConnector, "Suspending CU (waiting for EP %d)\n", wakeupEp);
    connector->suspend();

    return true;
}

void
TcuConnector::stopSleep()
{
    connector->wakeup();
}

void
TcuConnector::wakeupCore(bool force, epid_t rep)
{
    if (force || wakeupEp == Tcu::INVALID_EP_ID || rep == wakeupEp)
    {
        // better stop the command in this cycle to ensure that the core
        // does not issue another command before we can finish the sleep.
        if (tcu.regs().getCommand().opcode == CmdCommand::SLEEP)
            tcu.scheduleCmdFinish(Cycles(0));
        else
            connector->wakeup();
    }
}

void
TcuConnector::setIrq(BaseConnector::IRQ irq)
{
    wakeupCore(true, Tcu::INVALID_EP_ID);

    connector->setIrq(irq);

    irqInjects++;
}

void
TcuConnector::clearIrq(BaseConnector::IRQ irq)
{
    connector->clearIrq(irq);
}

void
TcuConnector::fireTimer()
{
    setIrq(BaseConnector::IRQ::TIMER);
}

void
TcuConnector::restartTimer(uint64_t nanos)
{
    if (fireTimerEvent.scheduled())
        tcu.deschedule(&fireTimerEvent);
    if (nanos != 0)
    {
        Cycles sleep_time = tcu.ticksToCycles(nanos * 1000);
        tcu.schedule(&fireTimerEvent, tcu.clockEdge(sleep_time));
    }
}

}
}
