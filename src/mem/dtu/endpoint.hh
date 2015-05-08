/*
 * Copyright (c) 2015, Christian Menard
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

#ifndef __MEM_DTU_BASE_HH__
#define __MEM_DTU_BASE_HH__

#include "mem/dtu/regfile.hh"

class Endpoint
{
  private:

    unsigned epid;

    RegFile& regFile;

    DtuReg readReg(EndpointRegister reg)
    {
        return regFile.readEpReg(epid, reg);
    }

    void writeReg(EndpointRegister reg, DtuReg value)
    {
        regFile.setEpReg(epid, reg, value);
    }

  public:

    bool isSender()
    {
        return readReg(EndpointRegister::CONFIG);
    }

    bool isReceiver()
    {
        return !isSender();
    }

    unsigned getEpid() { return epid; }

    unsigned getTargetEpid()
    {
        return readReg(EndpointRegister::TARGET_EPID);
    }

    unsigned getTargetCoreid()
    {
        return readReg(EndpointRegister::TARGET_COREID);
    }

    Addr getMessageAddr()
    {
        return readReg(EndpointRegister::MESSAGE_ADDR);
    }

    Addr getMessageSize()
    {
        return readReg(EndpointRegister::MESSAGE_SIZE);
    }

    Addr getBufferAddr()
    {
        return readReg(EndpointRegister::BUFFER_ADDR);
    }

    Addr getBufferSize()
    {
        return readReg(EndpointRegister::BUFFER_SIZE);
    }

    Endpoint(unsigned _epid, RegFile& regFile);
};

#endif // __MEM_DTU_ENDPOINT_HH__
