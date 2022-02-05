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

#ifndef __ARCH_M3_ARM_SYSTEM_HH__
#define __ARCH_M3_ARM_SYSTEM_HH__

#include <string>
#include <vector>

#include "arch/arm/system.hh"
#include "params/M3ArmSystem.hh"
#include "mem/qport.hh"
#include "mem/tcu/noc_addr.hh"
#include "sim/tile_memory.hh"
#include "sim/m3_loader.hh"

class M3ArmSystem : public ArmSystem, public TileMemory
{
    class NoCMasterPort : public QueuedRequestPort
    {
      protected:

        ReqPacketQueue reqQueue;

        SnoopRespPacketQueue snoopRespQueue;

      public:

        NoCMasterPort(M3ArmSystem &_sys);

        bool recvTimingResp(PacketPtr) override
        {
            // unused
            return true;
        }
    };

    NoCMasterPort nocPort;

  public:
    typedef M3ArmSystemParams Params;
    M3ArmSystem(const Params &p);

    uint32_t tileDesc(tileid_t tile) const override;

    Port& getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void initState();

  private:

    M3Loader loader;
};

#endif
