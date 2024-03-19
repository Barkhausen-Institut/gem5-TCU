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

#include "debug/TcuConnector.hh"
#include "mem/tcu/connector/base.hh"
#include "mem/tcu/tcu.hh"

namespace gem5
{
namespace tcu
{

void
BaseConnector::reset(bool start)
{
    while (!_pending.empty())
    {
        IRQ irq = _pending.front();
        DPRINTF(TcuConnector, "Forgetting IRQ %d\n", irq);
        _pending.pop();
    }
}

void
BaseConnector::setIrq(IRQ irq)
{
    _pending.push(irq);
    if (_pending.size() == 1)
        startIrq(irq);
    else
        DPRINTF(TcuConnector, "Delaying IRQ %d\n", irq);
}

bool
BaseConnector::clearIrq(IRQ *irq)
{
    if (_pending.size() == 0)
        return false;

    *irq = _pending.front();
    doClearIrq(*irq);

    _pending.pop();
    if (!_pending.empty())
    {
        DPRINTF(TcuConnector, "Starting delayed IRQ %d\n", _pending.front());
        startIrq(_pending.front());
    }
    return true;
}

void
BaseConnector::startIrq(IRQ irq)
{
    doSetIrq(irq);
}

}
}
