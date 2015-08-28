/*
 * Copyright (c) 2015, Nils Asmussen
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

#ifndef __DEV_IOPLATFORM_HH__
#define __DEV_IOPLATFORM_HH__

#include "dev/platform.hh"
#include "params/IOPlatform.hh"

class IOPlatform : public Platform
{
  public:
    typedef IOPlatformParams Params;

    IOPlatform(const Params *p) : Platform(p) {
    }

    /**
     * Cause the cpu to post a serial interrupt to the CPU.
     */
    virtual void postConsoleInt() {
    }

    /**
     * Clear a posted CPU interrupt
     */
    virtual void clearConsoleInt() {
    }

    /**
     * Cause the chipset to post a pci interrupt to the CPU.
     */
    virtual void postPciInt(int /* line */) {
    }

    /**
     * Clear a posted PCI->CPU interrupt
     */
    virtual void clearPciInt(int /* line */) {
    }

    virtual Addr pciToDma(Addr pciAddr) const {
        return pciAddr;
    }

    /**
     * Calculate the configuration address given a bus/dev/func.
     */
    virtual Addr calcPciConfigAddr(int /* bus */, int /* dev */, int /* func */) {
        return 0;
    }

    /**
     * Calculate the address for an IO location on the PCI bus.
     */
    virtual Addr calcPciIOAddr(Addr addr) {
        return addr;
    }

    /**
     * Calculate the address for a memory location on the PCI bus.
     */
    virtual Addr calcPciMemAddr(Addr addr) {
        return addr;
    }
};

#endif // __DEV_IOPLATFORM_HH__
