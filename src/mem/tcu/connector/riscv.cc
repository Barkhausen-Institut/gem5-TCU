/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2021 Nils Asmussen, Barkhausen Institut
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

#include "arch/riscv/faults.hh"
#include "cpu/simple/base.hh"
#include "debug/TcuConnector.hh"
#include "mem/tcu/connector/riscv.hh"
#include "mem/tcu/tcu.hh"
#include "sim/process.hh"

static int translate(RiscvConnector::IRQ irq)
{
    if (irq == RiscvConnector::CORE_REQ)
        return RiscvISA::ExceptionCode::INT_EXT_SUPER;
    return RiscvISA::ExceptionCode::INT_TIMER_SUPER;
}

RiscvConnector::RiscvConnector(const RiscvConnectorParams &p)
  : CoreConnector(p)
{
}

void
RiscvConnector::doSetIrq(IRQ irq)
{
    int vector = translate(irq);
    DPRINTF(TcuConnector, "Injecting IRQ %d (vector %d)\n", irq, vector);

    ThreadContext *tc = system->threads[0];
    tc->getCpuPtr()->getInterruptController(0)->post(vector, 0);
}

void
RiscvConnector::doClearIrq(IRQ irq)
{
    int vector = translate(irq);
    DPRINTF(TcuConnector, "Clearing IRQ %d (vector %d)\n", irq, vector);

    ThreadContext *tc = system->threads[0];
    tc->getCpuPtr()->getInterruptController(0)->clear(vector, 0);
}
