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

#include "sim/mem_system.hh"
#include "mem/port_proxy.hh"
#include "params/MemSystem.hh"

MemSystem::MemSystem(Params *p)
    : System(p), memFile(p->mem_file)
{
}

MemSystem::~MemSystem()
{
}

void
MemSystem::initState()
{
    System::initState();

    if(!memFile.empty())
    {
        FILE *f = fopen(memFile.c_str(), "r");
        if(!f)
            panic("Unable to open '%s' for reading", memFile.c_str());

        fseek(f, 0L, SEEK_END);
        size_t sz = ftell(f);
        fseek(f, 0L, SEEK_SET);

        auto data = new uint8_t[sz];
        if(fread(data, 1, sz, f) != sz)
            panic("Unable to read '%s'", memFile.c_str());
        physProxy.writeBlob(0x0, data, sz);
        delete[] data;
        fclose(f);
    }
}

MemSystem *
MemSystemParams::create()
{
    return new MemSystem(this);
}
