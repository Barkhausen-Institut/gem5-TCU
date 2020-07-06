/*
 * Copyright (c) 2016, Nils Asmussen
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
#include "mem/tcu/connector/core.hh"
#include "mem/tcu/tcu.hh"
#include "cpu/simple/base.hh"
#include "cpu/minor/cpu.hh"
#include "sim/process.hh"

CoreConnector::CoreConnector(const CoreConnectorParams *p)
  : BaseConnector(p),
    system(p->system)
{
}

void
CoreConnector::wakeup()
{
    if (system->threadContexts.size() == 0)
        return;

    if (system->threadContexts[0]->status() == ThreadContext::Suspended)
    {
        DPRINTF(TcuConnector, "Waking up core\n");
        system->threadContexts[0]->activate();
    }
}

void
CoreConnector::suspend()
{
    if (system->threadContexts.size() == 0)
        return;
    if (dynamic_cast<MinorCPU*>(system->threadContexts[0]->getCpuPtr()))
        return;

    if (system->threadContexts[0]->status() == ThreadContext::Active)
    {
        DPRINTF(TcuConnector, "Suspending core\n");
        system->threadContexts[0]->suspend();
    }
}

CoreConnector*
CoreConnectorParams::create()
{
    return new CoreConnector(this);
}
