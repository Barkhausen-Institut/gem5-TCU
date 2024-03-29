/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
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

#include "arch/arm/m3/system.hh"
#include "params/M3ArmSystem.hh"

#include <libgen.h>

namespace gem5
{

M3ArmSystem::NoCMasterPort::NoCMasterPort(M3ArmSystem &_sys)
  : QueuedRequestPort("noc_master_port", &_sys, reqQueue, snoopRespQueue),
    reqQueue(_sys, *this),
    snoopRespQueue(_sys, *this)
{}

M3ArmSystem::M3ArmSystem(const Params &p)
    : ArmSystem(p),
      TileMemory(this, p.memory_tile, p.memory_offset, p.memory_size,
                 physProxy),
      nocPort(*this),
      loader(p.tiles_per_chip, p.tiles, p.mods, p.cmdline, p.logflags,
             p.env_start, p.tile_id, p.mod_offset, p.mod_size, p.tile_size)
{
}

tcu::tiledesc_t M3ArmSystem::tileDesc(tcu::TileId tile) const
{
    return loader.tile_attr(tile);
}

Port&
M3ArmSystem::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "noc_master_port")
        return nocPort;
    return System::getPort(if_name, idx);
}

void
M3ArmSystem::initState()
{
    ArmSystem::initState();

    loader.initState(*this, *this, nocPort);

    workload->initState();
}

}
