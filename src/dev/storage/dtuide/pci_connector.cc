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
PCIConnector::reset(Addr addr)
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
