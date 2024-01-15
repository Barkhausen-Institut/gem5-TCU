/*
 * Copyright (C) 2022 Nils Asmussen, Barkhausen Institut
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

#ifndef __ARCH_M3_SYSTEM_HH__
#define __ARCH_M3_SYSTEM_HH__

#include <string>
#include <vector>

#include "params/M3System.hh"
#include "mem/qport.hh"
#include "mem/tcu/reg_file.hh"
#include "sim/tile_memory.hh"
#include "sim/m3_loader.hh"
#include "sim/system.hh"

namespace gem5
{

class M3System : public System, public TileMemory
{
    class NoCRequestPort : public QueuedRequestPort
    {
      protected:

        ReqPacketQueue reqQueue;

        SnoopRespPacketQueue snoopRespQueue;

      public:

        NoCRequestPort(M3System &_sys);

        bool recvTimingResp(PacketPtr) override
        {
            // unused
            return true;
        }
    };

    NoCRequestPort nocPort;

  public:
    typedef M3SystemParams Params;
    M3System(const Params &p);

    tcu::TileId tile() const
    {
        return loader.tileId;
    }

    bool started() const
    {
        return _started;
    }

    void set_started(bool started)
    {
        _started = started;
    }

    tcu::tiledesc_t tileDesc(tcu::TileId tile) const override;

    Port& getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void initState();

  private:

    M3Loader loader;

    bool _started;
};

}

#endif
