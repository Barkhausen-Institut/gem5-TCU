/*
 * Copyright (c) 2015, Christian Menard
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

#ifndef __SCRATCHPAD_HH_
#define __SCRATCHPAD_HH_

#include "mem/abstract_mem.hh"
#include "mem/tport.hh"
#include "params/Scratchpad.hh"

namespace gem5
{

namespace memory
{

class Scratchpad : public AbstractMemory
{
  private:

    class ScratchpadPort : public SimpleTimingPort
    {
      private:

        Scratchpad& scratchpad;

      public:

        ScratchpadPort(const std::string& _name, Scratchpad& _scratchpad);

      protected:

        Tick recvAtomic(PacketPtr pkt) override;

        void recvFunctional(PacketPtr pkt) override;

        AddrRangeList getAddrRanges() const override;
    };

    ScratchpadPort cpuPort;

    ScratchpadPort tcuPort;

    const Cycles latency;

    const unsigned throughput;

    const bool ignoreInvalid;

    bool checkInvalid(PacketPtr pkt);

  protected:

    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);

  public:

    Scratchpad(const ScratchpadParams &p);

    void init() override;

    Port& getPort(const std::string& if_name,
                  PortID idx = InvalidPortID) override;
};

}
}

#endif // __SCRATCHPAD_HH_
