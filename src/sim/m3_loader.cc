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

#include "sim/m3_loader.hh"
#include "base/trace.hh"
#include "base/loader/object_file.hh"
#include "cpu/thread_context.hh"
#include "debug/TcuTlb.hh"
#include "mem/port_proxy.hh"
#include "mem/tcu/tlb.hh"
#include "mem/tcu/tcu.hh"
#include "sim/byteswap.hh"
#include "sim/m3_system.hh"

#include <libgen.h>
#include <sstream>

namespace gem5
{

M3Loader::M3Loader(const std::vector<Addr> &tile_descs,
                   const std::vector<Addr> &tile_ids,
                   const std::vector<std::string> &mods,
                   const std::vector<std::string> &rot_layers,
                   const std::string &cmdline,
                   const std::string &logflags,
                   const std::string &kernel_cmdline,
                   Addr envStart,
                   tcu::TileId tileId,
                   Addr modOffset,
                   Addr modSize,
                   Addr tileSize)
    : tiles(),
      mods(mods),
      rotLayers(rot_layers),
      commandLine(cmdline),
      logflags(logflags),
      kernelCmdline(kernel_cmdline),
      envStart(envStart),
      tileId(tileId),
      modOffset(modOffset),
      modSize(modSize),
      tileSize(tileSize)
{
    assert(tile_descs.size() == tile_ids.size());
    for (size_t i = 0; i < tile_ids.size(); ++i)
        tiles[tcu::TileId::from_raw(tile_ids[i])] = tile_descs[i];
}

void
M3Loader::writeRemote(RequestPort &noc, Addr dest,
                      const uint8_t *data, size_t size)
{
    auto req = std::make_shared<Request>(dest, size, 0, Request::funcRequestorId);
    Packet pkt(req, MemCmd::WriteReq);
    pkt.dataStaticConst(data);

    auto senderState = new tcu::Tcu::NocSenderState();
    senderState->packetType = tcu::Tcu::NocPacketType::CACHE_MEM_REQ_FUNC;
    senderState->result = tcu::TcuError::NONE;

    pkt.pushSenderState(senderState);

    noc.sendFunctional(&pkt);

    delete senderState;
}

Addr
M3Loader::loadModule(RequestPort &noc, const std::string &filename, Addr addr)
{
    FILE *f = fopen(filename.c_str(), "r");
    if(!f)
        panic("Unable to open '%s' for reading", filename.c_str());

    fseek(f, 0L, SEEK_END);
    size_t sz = ftell(f);
    fseek(f, 0L, SEEK_SET);

    auto data = new uint8_t[sz];
    if(fread(data, 1, sz, f) != sz)
        panic("Unable to read '%s'", filename.c_str());
    writeRemote(noc, addr, data, sz);
    delete[] data;
    fclose(f);

    return sz;
}

void
M3Loader::writeArg(System &sys, Addr buf, size_t i, Addr argv,
                   const std::string &arg)
{
    // write argument pointer
    uint64_t argvPtr = buf;
    sys.physProxy.writeBlob(argv + i * sizeof(uint64_t),
                            (uint8_t*)&argvPtr, sizeof(argvPtr));
    if (buf)
    {
        // write argument
        sys.physProxy.writeBlob(buf, (uint8_t*)arg.c_str(), arg.length() + 1);
    }
}

Addr
M3Loader::writeArgs(System &sys, const std::vector<std::string> &args,
                    Addr argv, Addr bufStart, Addr bufEnd)
{
    size_t i = 0;
    Addr bufCur = bufStart;
    for (auto arg : args)
    {
        if(bufCur + arg.length() + 1 > bufEnd)
            panic("Not enough space for argv or envp.\n");
        writeArg(sys, bufCur, i, argv, arg);
        bufCur += arg.length() + 1;
        i++;
    }

    // null termination
    writeArg(sys, 0, i, argv, "");
    return bufCur;
}

Addr
M3Loader::loadModules(TileMemory &mem, RequestPort &noc, Addr offset,
                      BootModule *bmods)
{
    size_t i = 0;
    Addr addr = tcu::NocAddr(mem.memTile, offset).getAddr();
    for (const std::string &mod : mods)
    {
        // default --mods parameter leads to empty mod string
        if (mod.empty())
            continue;

        // split into name and path by "="
        std::stringstream ss(mod);
        std::string name, path;
        panic_if(!std::getline(ss, name, '='),
            "Unable to find '=' in module description");
        panic_if(!std::getline(ss, path),
            "Unable to read path from module description");

        Addr size = loadModule(noc, path, addr);

        // construct module info
        bmods[i].addr = addr;
        bmods[i].size = size;
        panic_if(name.length() >= MAX_MODNAME_LEN, "name too long");
        strcpy(bmods[i].name, name.c_str());

        inform("Loaded '%s' to %p .. %p",
            path, bmods[i].addr, bmods[i].addr + bmods[i].size);

        // to next
        addr += size + tcu::TcuTlb::PAGE_SIZE - 1;
        addr &= ~static_cast<Addr>(tcu::TcuTlb::PAGE_SIZE - 1);
        i++;
    }
    return addr;
}

void
M3Loader::initState(System &sys, TileMemory &mem, RequestPort &noc)
{
    // first initialize EP0 to a memory EP for our tile-local memory region in DRAM
    if (mem.initEpsAddr)
    {
        tcu::Ep ep;
        ep.mem.id = 0;
        ep.mem.r0.type = static_cast<tcu::RegFile::reg_t>(tcu::EpType::MEMORY);
        ep.mem.r0.act = tcu::Tcu::INVALID_ACT_ID;
        ep.mem.r0.flags = tcu::Tcu::MemoryFlags::READ | tcu::Tcu::MemoryFlags::WRITE;
        ep.mem.r0.targetTile = mem.memTile.raw();
        ep.mem.r1.remoteAddr = mem.memOffset;
        ep.mem.r2.remoteSize = mem.memSize;
        tcu::RegFile::printEpAccess(sys, ep, false, tcu::RegAccess::CPU);
        sys.physProxy.writeBlob(mem.initEpsAddr,
                    reinterpret_cast<const uint8_t*>(ep.inval.r), sizeof(ep.inval.r));
    }

    // init boot environment for the kernel
    BootEnv env;
    memset(&env, 0, sizeof(env));
    env.platform = Platform::GEM5;
    env.tile_id = tileId.raw();
    env.tile_desc = tile_attr(tileId);
    env.raw_tile_count = tiles.size();
    panic_if(tileId.chip() > MAX_CHIPS, "Exceeded maximum number of chips");
    panic_if(env.raw_tile_count > MAX_TILES, "Exceeded maximum number of tiles");
    // write tile ids to environment
    {
        size_t i = 0;
        for(auto it = tiles.cbegin(); it != tiles.cend(); ++it, ++i)
            env.raw_tile_ids[i] = it->first.raw();
    }

    // convert command line to an array of strings
    std::istringstream iss(commandLine);
    std::vector<std::string> args{std::istream_iterator<std::string>(iss),
                                  std::istream_iterator<std::string>()};
    // build environment (only containing LOG=<logflags>)
    std::vector<std::string> envs;
    envs.push_back(std::string("LOG=") + logflags);

    Addr argv = envStart + sizeof(env);
    // the kernel gets the kernel env behind the normal env
    if (modSize)
        argv += sizeof(KernelEnv);

    // calculate argc, argv, and envp
    env.argc = args.size();
    env.argv = argv;
    env.envp = argv + sizeof(uint64_t) * (env.argc + 1);
    Addr bufStart = env.envp + sizeof(uint64_t) * 2;

    // write arguments and argument pointer to memory
    bufStart = writeArgs(sys, args, argv, bufStart, envStart + ENV_SIZE);
    writeArgs(sys, envs, env.envp, bufStart, envStart + ENV_SIZE);

    if (!rotLayers.empty())
    {
        static_assert(sizeof(RotCfg) <= tcu::TcuTlb::PAGE_SIZE,
                      "RoT configuration too large");
        auto offset = modOffset + tcu::TcuTlb::PAGE_SIZE; // For RotCfg
        RotBinary rotBins[3];
        panic_if(rotLayers.size() != 3, "Unexpected number of RoT layers");

        size_t i = 0;
        for (const std::string &layer : rotLayers)
        {
            Addr addr = tcu::NocAddr(mem.memTile, offset).getAddr();
            Addr size = loadModule(noc, layer, addr);
            rotBins[i].flash_offset = offset;
            rotBins[i].size = size;

            inform("Loaded RoT layer '%s' to %p .. %p",
                   layer, addr, addr + size);

            // to next
            offset += size + tcu::TcuTlb::PAGE_SIZE - 1;
            offset &= ~static_cast<Addr>(tcu::TcuTlb::PAGE_SIZE - 1);
            i++;
        }

        constexpr size_t TCU_EPS_PER_PAGE = tcu::TcuTlb::PAGE_SIZE /
            (tcu::numEpRegs * sizeof(tcu::RegFile::reg_t));
        RotCfg cfg = {
            .brom = {
                .magic = 0x42726f6d43666701,
                .next_layer = rotBins[0],
            },
            .blau = {
                .magic = 0x426c617543666701,
                .next_layer = rotBins[1],
            },
            .rosa = {
                .magic = 0x526f736143666702,
                .kernel_mem_size = tileSize,
                .kernel_ep_pages = static_cast<uint8_t>(
                    // For now just configure the kernel tile with the same
                    // amount of EPs as used for the RoT tile (could also make
                    // this separately configurable).
                    std::min(divCeil(mem.initEpsNum, TCU_EPS_PER_PAGE),
                             static_cast<size_t>(UINT8_MAX))),
                .kernel_cmdline = {},
                .mods = {},
                .next_layer = rotBins[2],
            },
        };

        panic_if(mods.size() > (sizeof(cfg.rosa.mods) / sizeof(cfg.rosa.mods[0])),
                 "Too many modules for RoT");
        panic_if(kernelCmdline.length() >= sizeof(cfg.rosa.kernel_cmdline) - 1,
                 "Kernel command line too long");
        strcpy(cfg.rosa.kernel_cmdline, kernelCmdline.c_str());

        Addr addr = loadModules(mem, noc, offset, cfg.rosa.mods);
        Addr end = tcu::NocAddr(mem.memTile, modOffset + modSize).getAddr();
        panic_if(addr > end, "Modules are too large (have: %lu, need: %lu)",
                 modSize, addr - tcu::NocAddr(mem.memTile, modOffset).getAddr());

        writeRemote(noc, tcu::NocAddr(mem.memTile, modOffset).getAddr(),
                    reinterpret_cast<uint8_t*>(&cfg), sizeof(cfg));
    }
    // modules for the kernel
    else if (modSize)
    {
        BootModule *bmods = new BootModule[mods.size()]();
        Addr addr = loadModules(mem, noc, modOffset, bmods);

        // determine memory regions
        size_t mem_count = 0;
        MemMod *bmems = new MemMod[tiles.size()];
        auto avail_mem_start = std::max(modOffset + modSize + tileSize,
                                        mem.memOffset + mem.memSize);
        bmems[0].size = (tile_attr(mem.memTile) >> 28) << 12;
        if (bmems[0].size < avail_mem_start)
            panic("Not enough DRAM for modules and tiles");
        bmems[0].addr = tcu::NocAddr(mem.memTile, avail_mem_start).getAddr();
        bmems[0].size -= avail_mem_start;
        mem_count++;

        for(auto it = tiles.cbegin(); it != tiles.cend(); ++it)
        {
            if (it->first != mem.memTile && (tile_attr(it->first) & 0x3F) == 1) {
                bmems[mem_count].addr = tcu::NocAddr(it->first, 0).getAddr();
                bmems[mem_count].size = (tile_attr(it->first) >> 28) << 12;
                mem_count++;
            }
        }

        // write kenv
        env.kenv = addr;
        KernelEnv kenv;
        kenv.mod_count = mods.size();
        kenv.tile_count  = tiles.size();
        kenv.mem_count = mem_count;
        kenv.serv_count = 0;
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(&kenv),
                    sizeof(kenv));
        addr += sizeof(kenv);

        // write modules to memory
        size_t bmodsize = kenv.mod_count * sizeof(BootModule);
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(bmods), bmodsize);
        delete[] bmods;
        addr += bmodsize;

        // write tile descriptors to memory
        uint64_t *ktileDescs = new uint64_t[kenv.tile_count]();
        {
            size_t i = 0;
            for(auto it = tiles.cbegin(); it != tiles.cend(); ++it, ++i)
                ktileDescs[i] = it->second;
        }
        size_t bdescssize = kenv.tile_count * sizeof(uint64_t);
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(ktileDescs), bdescssize);
        delete[] ktileDescs;
        addr += bdescssize;

        // write memory regions to memory
        size_t bmemsize = kenv.mem_count * sizeof(MemMod);
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(bmems), bmemsize);
        delete[] bmems;
        addr += bmemsize;

        // check size
        Addr end = tcu::NocAddr(mem.memTile, modOffset + modSize).getAddr();
        if (addr > end)
        {
            panic("Modules are too large (have: %lu, need: %lu)",
                modSize, addr - tcu::NocAddr(mem.memTile, modOffset).getAddr());
        }
    }

    // write env
    sys.physProxy.writeBlob(
        envStart, reinterpret_cast<uint8_t*>(&env), sizeof(env));
}

}
