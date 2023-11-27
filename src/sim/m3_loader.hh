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

#ifndef __SIM_M3_LOADER_HH__
#define __SIM_M3_LOADER_HH__

#include <map>
#include <string>
#include <vector>

#include "sim/system.hh"
#include "mem/tcu/reg_file.hh"
#include "sim/tile_memory.hh"

namespace gem5
{

class M3Loader
{
  protected:
    static const size_t ENV_SIZE        = 0x1000;
    static const size_t HEAP_SIZE       = 64 * 0x1000;
    static const size_t MAX_MODNAME_LEN = 64;
    static const size_t MAX_CHIPS       = 2;
    static const size_t MAX_TILES       = 64;

    struct M5_ATTR_PACKED BootModule
    {
        uint64_t addr;
        uint64_t size;
        char name[MAX_MODNAME_LEN];
    };

    struct M5_ATTR_PACKED MemMod
    {
        uint64_t addr;
        uint64_t size;
    };

    struct M5_ATTR_PACKED KernelEnv
    {
        uint64_t mod_count;
        uint64_t tile_count;
        uint64_t mem_count;
        uint64_t serv_count;
    };

    enum Platform
    {
        GEM5,
        HW
    };

    struct M5_ATTR_PACKED BootEnv
    {
        uint64_t platform;
        uint64_t tile_id;
        uint64_t tile_desc;
        uint64_t argc;
        uint64_t argv;
        uint64_t envp;
        uint64_t kenv;
        uint64_t raw_tile_count;
        uint64_t raw_tile_ids[MAX_CHIPS * MAX_TILES];
    };

    std::map<tcu::TileId, tcu::tiledesc_t> tiles;
    std::vector<std::string> mods;
    std::string commandLine;
    std::string logflags;

  public:
    const Addr envStart;
    const tcu::TileId tileId;
    const Addr modOffset;
    const Addr modSize;
    const Addr tileSize;

  public:
    M3Loader(const std::vector<Addr> &tile_descs,
             const std::vector<Addr> &tile_ids,
             const std::vector<std::string> &mods,
             const std::string &cmdline,
             const std::string &logflags,
             Addr envStart,
             tcu::TileId tileId,
             Addr modOffset,
             Addr modSize,
             Addr tileSize);

    tcu::tiledesc_t tile_attr(tcu::TileId tileId) const
    {
        return tiles.at(tileId);
    }

    void initState(System &sys, TileMemory &mem, RequestPort &noc);

  private:
    void writeArg(System &sys, Addr buf, size_t i, Addr argv,
                  const std::string &arg);
    Addr writeArgs(System &sys, const std::vector<std::string> &args,
                   Addr argv, Addr bufStart, Addr bufEnd);
    void writeRemote(RequestPort &noc, Addr dest,
                     const uint8_t *data, size_t size);
    Addr loadModule(RequestPort &noc, const std::string &filename, Addr addr);
    Addr loadModules(TileMemory &mem, RequestPort &noc, Addr offset,
                     BootModule *bmods);
};

}

#endif
