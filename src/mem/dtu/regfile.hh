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

#ifndef __MEM_DTU_REGFILE_HH__
#define __MEM_DTU_REGFILE_HH__

#include <vector>

#include "base/types.hh"
#include "mem/packet.hh"

enum class DtuReg : Addr
{
    COMMAND,
    STATUS,
};

constexpr unsigned numDtuRegs = 2;

enum class EpReg : Addr
{
    MODE,
    MAX_MESSAGE_SIZE,
    // for receiving messages
    BUFFER_MESSAGE_COUNT,
    BUFFER_ADDR,
    BUFFER_SIZE,
    BUFFER_READ_PTR,
    BUFFER_WRITE_PTR,
    // for sending messages
    TARGET_COREID,
    TARGET_EPID,
    MESSAGE_ADDR,
    MESSAGE_SIZE,
    REPLY_EPID,
    // for memory requests
    REQUEST_LOCAL_ADDR,
    REQUEST_REMOTE_ADDR,
    REQUEST_SIZE,
};

constexpr unsigned numEpRegs = 15;

class RegFile
{
  public:

    using reg_t = uint32_t;

    static Addr getRegAddr(DtuReg reg);

    static Addr getRegAddr(EpReg reg, unsigned epid);

  private:

    std::vector<reg_t> dtuRegs;

    std::vector<std::vector<reg_t>> epRegs;

    const unsigned numEndpoints;

    // used for debug messages (DPRINTF)
    const std::string _name;

  public:

    RegFile(const std::string name, unsigned numEndpoints);

    reg_t readDtuReg(DtuReg reg) const;

    void setDtuReg(DtuReg reg, reg_t value);

    reg_t readEpReg(unsigned epid, EpReg reg) const;

    void setEpReg(unsigned epid, EpReg reg, reg_t value);

    /// returns true if the command register was written
    bool handleRequest(PacketPtr pkt);

    const std::string name() const { return _name; }

    Addr getSize() const;
};

#endif // __MEM_DTU_REGFILE_HH__
