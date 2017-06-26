
#ifndef __DEV_STORAGE_DTU_IDE_DTU_PLATFORM_HH__
#define __DEV_STORAGE_DTU_IDE_DTU_PLATFORM_HH__

#include "dev/platform.hh"
#include "dev/storage/dtuide/base_proxy.hh"
#include "params/DtuPlatform.hh"

class DtuPlatform : public Platform {

  private:
        BaseProxy * idedtuproxy;

  public:
        DtuPlatform(const DtuPlatformParams * p);
        ~DtuPlatform() override;

    /**
     * Cause the cpu to post a serial interrupt to the CPU.
     */
    void postConsoleInt() override;

    /**
     * Clear a posted CPU interrupt
     */
    void clearConsoleInt() override;


    /**
     * Cause the chipset to post a cpi interrupt to the CPU.
     */
    void postPciInt(int line) override;

    /**
     * Clear a posted PCI->CPU interrupt
     */
    void clearPciInt(int line) override;
};

#endif
