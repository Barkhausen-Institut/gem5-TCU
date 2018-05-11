/*
 * Copyright (c) 2018, Lukas Landgraf
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

#include "dev/storage/dtuide/pci_connector.hh"

#include "debug/PCIConnector.hh"
#include "dev/storage/dtuide/base_proxy.hh"

PCIConnector::PCIConnector(const PCIConnectorParams *p)
  : BaseConnector(p),
    proxy(p->base_proxy)
{
    proxy->setConnector(this);
}

void
PCIConnector::setIrq()
{
    DPRINTF(PCIConnector, "Sending interrupt signal to accelerator\n");
    proxy->interruptFromConnector();
}

void
PCIConnector::reset(Addr, Addr)
{
    DPRINTF(PCIConnector, "Resetting accelerator\n");
    proxy->reset();
}

void
PCIConnector::wakeup()
{
    DPRINTF(PCIConnector, "Waking up accelerator\n");
    proxy->wakeup();
}

void
PCIConnector::suspend()
{
    DPRINTF(PCIConnector, "Suspending accelerator\n");
}

PCIConnector*
PCIConnectorParams::create()
{
    return new PCIConnector(this);
}
