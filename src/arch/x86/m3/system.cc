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

#include "arch/x86/m3/system.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/isa_traits.hh"
#include "arch/vtophys.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"
#include "mem/dtu/noc_addr.hh"
#include "mem/dtu/pt_unit.hh"
#include "mem/dtu/tlb.hh"
#include "params/M3X86System.hh"
#include "sim/byteswap.hh"

using namespace LittleEndianGuest;
using namespace X86ISA;

M3X86System::M3X86System(Params *p)
    : X86System(p),
      commandLine(p->boot_osflags),
      memPe(p->memory_pe),
      memOffset(p->memory_offset),
      memSize(p->memory_size)
{
}

M3X86System::~M3X86System()
{
}

size_t
M3X86System::getArgc() const
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
M3X86System::writeArg(Addr &args, size_t &i, Addr argv, const char *cmd, const char *begin) const
{
    const char zero[] = {0};
    // write argument pointer
    uint64_t argvPtr = args;
    physProxy.writeBlob(argv + i * sizeof(uint64_t), (uint8_t*)&argvPtr, sizeof(argvPtr));
    // write argument
    physProxy.writeBlob(args, (uint8_t*)begin, cmd - begin);
    args += cmd - begin;
    physProxy.writeBlob(args, (uint8_t*)zero, 1);
    args++;
    i++;
}

void
M3X86System::createPTEs() const
{
    // create page-table entries (atm, we have a single large pt, sitting at address 0)
    Addr phys = NocAddr(memPe, 0, memOffset).getAddr();
    size_t offset = 0;
    size_t count = divCeil(memSize, DtuTlb::PAGE_SIZE);
    for(size_t i = 0; i < count; ++i)
    {
        PtUnit::PageTableEntry e(0);
        e.base = NocAddr(memPe, 0, memOffset + offset).getAddr() >> DtuTlb::PAGE_BITS;
        if(!(offset >= 0x100000 && offset < 0x200000))
        {
            e.r = 1;
            e.w = 1;
            e.x = 1;
        }
        physProxy.write(phys, e);

        offset += DtuTlb::PAGE_SIZE;
        phys += sizeof(e);
    }
}

void
M3X86System::initState()
{
    X86System::initState();

    createPTEs();

    const Addr stateSize = 0x1000;
    // TODO
    const Addr stateEnd = memSize - 0x2000;
    const Addr stateArea = stateEnd - stateSize;

    // write argc and argv
    uint64_t argc = getArgc();
    uint64_t argv = stateArea + 2 * sizeof(uint64_t);
    physProxy.writeBlob(stateArea + 0, (uint8_t*)&argc, sizeof(argc));
    physProxy.writeBlob(stateArea + sizeof(uint64_t), (uint8_t*)&argv, sizeof(argv));

    Addr args = stateArea + (2 + argc + 1) * sizeof(uint64_t);

    // check if there is enough space
    if (commandLine.length() + 1 > stateEnd - args)
        panic("Command line \"%s\" is longer than %d characters.\n",
                commandLine, stateEnd - args - 1);

    // write arguments to state area
    const char *cmd = commandLine.c_str();
    const char *begin = cmd;
    size_t i = 0;
    while (*cmd)
    {
        if (isspace(*cmd))
        {
            if (cmd > begin)
                writeArg(args, i, argv, cmd, begin);
            begin = cmd + 1;
        }
        cmd++;
    }
    if (cmd > begin)
        writeArg(args, i, argv, cmd, begin);
}

M3X86System *
M3X86SystemParams::create()
{
    return new M3X86System(this);
}
