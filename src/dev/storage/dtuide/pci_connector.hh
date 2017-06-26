#ifndef __DEV_STORAGE_DTU_IDE_BASE_CONNECTOR_HH__
#define __DEV_STORAGE_DTU_IDE_BASE_CONNECTOR_HH__

#include "mem/dtu/connector/base.hh"
#include "params/PCIConnector.hh"

//#include "sim/system.hh"

class PCIConnector : public BaseConnector
{
  public:

    PCIConnector(const PCIConnectorParams *p);

    void setIrq() override;

    void reset(Addr addr) override;

    void wakeup() override;

    void suspend() override;

  private:

    BaseProxy * proxy;
};

#endif
