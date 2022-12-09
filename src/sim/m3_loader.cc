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

#include <libgen.h>
#include <sstream>

namespace gem5
{

M3Loader::M3Loader(const std::vector<Addr> &tile_descs,
                   const std::vector<Addr> &tile_ids,
                   const std::vector<std::string> &mods,
                   const std::string &cmdline,
                   Addr envStart,
                   tcu::TileId tileId,
                   Addr modOffset,
                   Addr modSize,
                   Addr tileSize)
    : tiles(),
      mods(mods),
      commandLine(cmdline),
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

size_t
M3Loader::getArgc() const
{
    const char *cmd = commandLine.c_str();
    size_t argc = 0;
    size_t len = 0;
    while (*cmd)
    {
        if (isspace(*cmd))
        {
            if(len > 0)
                argc++;
            len = 0;
        }
        else
            len++;
        cmd++;
    }
    if(len > 0)
        argc++;

    return argc;
}

void
M3Loader::writeArg(System &sys, Addr &args, size_t &i, Addr argv,
                   const char *cmd, const char *begin)
{
    const char zero[] = {0};
    // write argument pointer
    uint64_t argvPtr = args;
    sys.physProxy.writeBlob(argv + i * sizeof(uint64_t),
                            (uint8_t*)&argvPtr, sizeof(argvPtr));
    // write argument
    sys.physProxy.writeBlob(args, (uint8_t*)begin, cmd - begin);
    args += cmd - begin;
    sys.physProxy.writeBlob(args, (uint8_t*)zero, 1);
    args++;
    i++;
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
M3Loader::initState(System &sys, TileMemory &mem, RequestPort &noc)
{
    BootEnv env;
    memset(&env, 0, sizeof(env));
    env.platform = Platform::GEM5;
    env.tile_id = tileId.raw();
    env.tile_desc = tile_attr(tileId);
    env.argc = getArgc();
    Addr argv = envStart + sizeof(env);
    // the kernel gets the kernel env behind the normal env
    if (modOffset)
        argv += sizeof(KernelEnv);
    Addr args = argv + sizeof(uint64_t) * env.argc;
    env.argv = argv;

    // with paging, the kernel gets an initial heap mapped
    if ((env.tile_desc & 0x7) == 1 || (env.tile_desc & 0x7) == 2)
        env.heap_size = HEAP_SIZE;
    // otherwise, he should use all internal memory
    else
        env.heap_size = 0;

    // check if there is enough space
    if (commandLine.length() + 1 > envStart + ENV_SIZE - args)
    {
        panic("Command line \"%s\" is longer than %d characters.\n",
                commandLine, envStart + ENV_SIZE - args - 1);
    }

    // write arguments to state area
    const char *cmd = commandLine.c_str();
    const char *begin = cmd;
    size_t i = 0;
    while (*cmd)
    {
        if (isspace(*cmd))
        {
            if (cmd > begin)
                writeArg(sys, args, i, argv, cmd, begin);
            begin = cmd + 1;
        }
        cmd++;
    }

    if (cmd > begin)
        writeArg(sys, args, i, argv, cmd, begin);

    // modules for the kernel
    if (modOffset)
    {
        BootModule *bmods = new BootModule[mods.size()]();

        i = 0;
        Addr addr = tcu::NocAddr(mem.memTile, modOffset).getAddr();
        for (const std::string &mod : mods)
        {
            Addr size = loadModule(noc, mod, addr);

            // determine module name
            char *tmp = new char[mod.length() + 1];
            strcpy(tmp, mod.c_str());
            std::string mod_name(basename(tmp));
            delete[] tmp;

            // construct module info
            bmods[i].addr = addr;
            bmods[i].size = size;
            panic_if(mod_name.length() >= MAX_MODNAME_LEN, "name too long");
            strcpy(bmods[i].name, mod_name.c_str());

            inform("Loaded '%s' to %p .. %p",
                bmods[i].name, bmods[i].addr, bmods[i].addr + bmods[i].size);

            // to next
            addr += size + tcu::TcuTlb::PAGE_SIZE - 1;
            addr &= ~static_cast<Addr>(tcu::TcuTlb::PAGE_SIZE - 1);
            i++;
        }

        // determine memory regions
        size_t mem_count = 0;
        MemMod *bmems = new MemMod[tiles.size()];
        auto avail_mem_start = modOffset + modSize + tiles.size() * tileSize;
        bmems[0].size = tile_attr(mem.memTile) & ~static_cast<Addr>(0xFFF);
        if (bmems[0].size < avail_mem_start)
            panic("Not enough DRAM for modules and tiles");
        bmems[0].addr = tcu::NocAddr(mem.memTile, avail_mem_start).getAddr();
        bmems[0].size -= avail_mem_start;
        mem_count++;

        for(auto it = tiles.cbegin(); it != tiles.cend(); ++it)
        {
            if (it->first != mem.memTile && (tile_attr(it->first) & 0x7) == 2) {
                bmems[mem_count].addr = tcu::NocAddr(it->first, 0).getAddr();
                bmems[mem_count].size = tile_attr(it->first) & ~static_cast<Addr>(0xFFF);
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

        // write tile ids to memory
        uint16_t *ktileIds = new uint16_t[kenv.tile_count]();
        {
            size_t i = 0;
            for(auto it = tiles.cbegin(); it != tiles.cend(); ++it, ++i)
                ktileIds[i] = it->first.raw();
        }
        size_t bidssize = kenv.tile_count * sizeof(uint16_t);
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(ktileIds), bidssize);
        delete[] ktileIds;
        addr += bidssize;

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
