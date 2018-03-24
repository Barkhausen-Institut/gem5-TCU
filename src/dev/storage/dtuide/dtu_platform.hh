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

#ifndef __DEV_STORAGE_DTU_IDE_DTU_PLATFORM_HH__
#define __DEV_STORAGE_DTU_IDE_DTU_PLATFORM_HH__

#include "dev/platform.hh"
#include "dev/storage/dtuide/base_proxy.hh"
#include "params/DtuPlatform.hh"

class DtuPlatform : public Platform
{
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
