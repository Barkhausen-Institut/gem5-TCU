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

#ifndef __SIM_M3_LOADER_HH__
#define __SIM_M3_LOADER_HH__

#include <string>
#include <vector>

#include "sim/system.hh"
#include "mem/dtu/noc_addr.hh"
#include "sim/dtu_memory.hh"

class M3Loader
{
  protected:
    static const size_t MAX_MODS        = 64;
    static const size_t MAX_PES         = 64;
    static const size_t RT_SIZE         = 0x2000;
    static const uintptr_t RT_START     = 0x6000;
    static const size_t STACK_SIZE      = 0x1000;
    static const uintptr_t STACK_AREA   = RT_START + RT_SIZE;
    static const size_t HEAP_SIZE       = 0x1000;

    struct BootModule
    {
        char name[256];
        uint64_t addr;
        uint64_t size;
    } M5_ATTR_PACKED;

    struct KernelEnv
    {
        enum
        {
            TYPE_IMEM    = 0,
            TYPE_EMEM    = 1,
            TYPE_MEM     = 2,
        };

        uint64_t mods[MAX_MODS];
        uint64_t pe_count;
        uint32_t pes[MAX_PES];
    } M5_ATTR_PACKED;

    struct StartEnv
    {
        uint64_t coreid;
        uint32_t argc;
        uint64_t argv;

        uint64_t sp;
        uint64_t entry;
        uint64_t lambda;
        uint32_t pager_sess;
        uint32_t pager_sgate;
        uint32_t pager_rgate;
        uint32_t mounts_len;
        uint64_t mounts;
        uint32_t fds_len;
        uint64_t fds;
        uint64_t rbufcur;
        uint64_t rbufend;
        uint64_t eps;
        uint64_t caps;
        uint64_t exit;
        uint64_t heapsize;

        uint64_t backend;
        uint64_t kenv;
        uint32_t pe;

        uint64_t isr64_handler;
    } M5_ATTR_PACKED;

    std::vector<Addr> pes;
    std::string commandLine;

  public:
    static const unsigned RES_PAGES;

    const unsigned coreId;
    const Addr modOffset;
    const Addr modSize;

  public:
    M3Loader(const std::vector<Addr> &pes,
             const std::string &cmdline,
             unsigned coreId,
             Addr modOffset,
             Addr modSize);

    const std::vector<Addr> &pe_attr() const
    {
        return pes;
    }

    void initState(System &sys, DTUMemory &dtumem, MasterPort &noc);

  private:
    bool isKernelArg(const std::string &arg);
    void mapMemory(System &sys, DTUMemory &dtumem);
    size_t getArgc() const;
    void writeArg(System &sys, Addr &args, size_t &i, Addr argv, const char *cmd, const char *begin);
    void writeRemote(MasterPort &noc, Addr dest, const uint8_t *data, size_t size);
    Addr loadModule(MasterPort &noc, const std::string &path, const std::string &name, Addr addr);
};

#endif
