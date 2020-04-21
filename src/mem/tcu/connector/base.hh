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

#ifndef __MEM_TCU_BASE_CONNECTOR__
#define __MEM_TCU_BASE_CONNECTOR__

#include "params/BaseConnector.hh"
#include "mem/mem_object.hh"
#include "sim/system.hh"

#include <queue>

class Tcu;

class BaseConnector : public ClockedObject
{
  public:

    enum IRQ
    {
        CORE_REQ,
        TIMER,
    };

    BaseConnector(const BaseConnectorParams *p)
      : ClockedObject(p),
        _tcu()
    { }

    void setTcu(Tcu *tcu) {
        _tcu = tcu;
    }

    // wakeup and suspend are only used to improve simulation speed.
    virtual void wakeup() {};
    virtual void suspend() {};

    virtual void reset() {};

    void setIrq(IRQ irq);
    void clearIrq(IRQ irq);

  private:

    void startIrq(IRQ irq);
    virtual void doSetIrq(IRQ) {};
    virtual void doClearIrq(IRQ) {};

    Tcu *_tcu;
    std::queue<IRQ> _pending;
};

#endif // __MEM_TCU_BASE_CONNECTOR__
