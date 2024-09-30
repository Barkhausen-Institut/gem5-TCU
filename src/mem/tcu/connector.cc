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
#include "reg_file.hh"

namespace gem5
{
namespace tcu
{

TcuConnector::TcuConnector(Tcu &_tcu, BaseConnector *_connector)
    : tcu(_tcu),
      connector(_connector),
      sleepEPs(tcu.eps().newCache(EpFile::Messaging, "sleep")),
      wakeupEp(Tcu::INVALID_EP_ID),
      invalidateEp(Tcu::INVALID_EP_ID),
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
    epid_t rep = cmd.arg0 & 0xFFFF;
    epid_t iep = (cmd.arg0 >> 16) & 0xFFFF;

    if (rep == Tcu::INVALID_EP_ID && iep == Tcu::INVALID_EP_ID)
    {
        if (tcu.regs().getAct(PrivReg::CUR_ACT).msgs > 0)
        {
            tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
            return;
        }

        if (!startSleep(rep, iep))
            tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
    }
    else
    {
        if (rep != Tcu::INVALID_EP_ID)
            sleepEPs.addEp(rep);
        if (iep != Tcu::INVALID_EP_ID)
            sleepEPs.addEp(iep);
        sleepEPs.onFetched(std::bind(&TcuConnector::startWaitEPWithEPs,
                                     this, std::placeholders::_1, rep, iep));
    }
}

void
TcuConnector::startWaitEPWithEPs(EpFile::EpCache &eps, epid_t rep, epid_t iep)
{
    if (rep != Tcu::INVALID_EP_ID)
    {
        const Ep ep = eps.getEp(rep);
        if (ep.type() == EpType::RECEIVE && ep.recv.unreadMsgs() > 0)
        {
            tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
            return;
        }
    }

    if (iep != Tcu::INVALID_EP_ID)
    {
        const Ep ep = eps.getEp(iep);
        if (ep.type() == EpType::INVALID)
        {
            tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
            return;
        }
    }

    if (!startSleep(rep, iep))
        tcu.scheduleCmdFinish(Cycles(1), TcuError::NONE);
}

bool
TcuConnector::startSleep(epid_t rep, epid_t iep, bool force)
{
    if (!force && connector->havePendingIrq())
        return false;

    wakeupEp = rep;
    invalidateEp = iep;
    DPRINTF(TcuConnector, "Suspending CU (REP=%d, IEP=%d)\n", wakeupEp, invalidateEp);
    connector->suspend();

    return true;
}

void
TcuConnector::stopSleep()
{
    wakeupEp = Tcu::INVALID_EP_ID;
    invalidateEp = Tcu::INVALID_EP_ID;
    connector->wakeup();
}

void
TcuConnector::wakeupForMsg(epid_t ep)
{
    if (wakeupEp == Tcu::INVALID_EP_ID || ep == wakeupEp)
        wakeup();
}

void
TcuConnector::wakeupForInvEp(epid_t ep)
{
    if (ep == invalidateEp)
        wakeup();
}

void
TcuConnector::wakeup()
{
    // better stop the command in this cycle to ensure that the core
    // does not issue another command before we can finish the sleep.
    if (tcu.regs().getCommand().opcode == CmdCommand::SLEEP)
        tcu.scheduleCmdFinish(Cycles(0));
    stopSleep();
}

void
TcuConnector::setIrq(BaseConnector::IRQ irq)
{
    wakeup();

    connector->setIrq(irq);

    irqInjects++;
}

bool
TcuConnector::clearIrq(BaseConnector::IRQ *irq)
{
    return connector->clearIrq(irq);
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
