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

#ifndef __ARCH_M3_X86_SYSTEM_HH__
#define __ARCH_M3_X86_SYSTEM_HH__

#include <string>
#include <vector>

#include "arch/x86/system.hh"
#include "params/M3X86System.hh"
#include "mem/dtu/noc_addr.hh"

class M3X86System : public X86System
{
  protected:
    static const size_t MAX_MODS        = 8;
    static const size_t RT_SIZE         = 0x2000;
    static const uintptr_t RT_START     = 0x1000;
    static const size_t STACK_SIZE      = 0x1000;
    static const uintptr_t STACK_AREA   = RT_START + RT_SIZE;
    static const size_t HEAP_SIZE       = 64 * 1024;
    static const unsigned RES_PAGES;

    struct BootModule
    {
        char name[128];
        uint64_t addr;
        uint64_t size;
    } M5_ATTR_PACKED;

    struct StartEnv
    {
        uint64_t coreid;
        uint32_t argc;
        char **argv;
        uintptr_t mods[MAX_MODS];

        uintptr_t sp;
        uintptr_t entry;
        uintptr_t lambda;
        uint32_t pager_sess;
        uint32_t pager_gate;
        uint32_t mounts_len;
        uintptr_t mounts;
        uint32_t fds_len;
        uintptr_t fds;
        uintptr_t eps;
        uintptr_t caps;
        uintptr_t exit;

        uintptr_t backend;
    } M5_ATTR_PACKED;

    std::string commandLine;

  public:
    const unsigned coreId;
    const unsigned memPe;
    const Addr memOffset;
    const Addr memSize;
    const Addr modOffset;
    const Addr modSize;
    unsigned nextFrame;

  public:
    typedef M3X86SystemParams Params;
    M3X86System(Params *p);
    ~M3X86System();

    NocAddr getRootPt() const
    {
        return NocAddr(memPe, 0, memOffset);
    }

    void initState();

  private:
    bool isKernelArg(const std::string &arg);
    void mapPage(Addr virt, Addr phys, uint access);
    void mapSegment(Addr start, Addr size, unsigned perm);
    void mapMemory();
    size_t getArgc() const;
    void writeArg(Addr &args, size_t &i, Addr argv, const char *cmd, const char *begin) const;
    Addr loadModule(const std::string &path, const std::string &name, Addr addr) const;
};

#endif
