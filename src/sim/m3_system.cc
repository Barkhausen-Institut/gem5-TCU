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

#include "mem/tcu/reg_file.hh"
#include "params/M3System.hh"
#include "sim/m3_system.hh"

#include <libgen.h>

namespace gem5
{

M3System::NoCRequestPort::NoCRequestPort(M3System &_sys)
  : QueuedRequestPort("noc_master_port", reqQueue, snoopRespQueue),
    reqQueue(_sys, *this),
    snoopRespQueue(_sys, *this)
{}

M3System::M3System(const Params &p)
    : System(p),
      TileMemory(this, tcu::TileId::from_raw(p.memory_tile),
                 p.memory_offset, p.memory_size, physProxy,
                 p.eps_addr,
                 p.eps_num),
      nocPort(*this),
      loader(p.tile_descs, p.tile_ids, p.mods, p.rot_layers, p.cmdline,
             p.logflags, p.kernel_cmdline, p.env_start,
             tcu::TileId::from_raw(p.tile_id),
             p.mod_offset, p.mod_size, p.tile_size),
      _started(!p.cmdline.empty())
{
}

tcu::tiledesc_t M3System::tileDesc(tcu::TileId tile) const
{
    return loader.tile_attr(tile);
}

Port&
M3System::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "noc_master_port")
        return nocPort;
    return System::getPort(if_name, idx);
}

void
M3System::initState()
{
    System::initState();

    if (_started)
    {
        loader.initState(*this, *this, nocPort);

        for (auto *tc: threads)
            tc->activate();
    }
}

}
