/*
 * Copyright (c) 2016, Nils Asmussen
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

#ifndef __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__
#define __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__

#include "params/DtuAccelHash.hh"
#include "cpu/dtu-accel-hash/algorithm.hh"
#include "mem/dtu/regfile.hh"
#include "sim/system.hh"

class DtuAccelHash : public MemObject
{
  public:
    DtuAccelHash(const DtuAccelHashParams *p);

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    void interrupt();

    void wakeup();

  protected:

    /// main simulation loop
    void tick();

    class CpuPort : public MasterPort
    {
      private:
        DtuAccelHash& dtutest;
      public:
        CpuPort(const std::string& _name, DtuAccelHash* _dtutest)
            : MasterPort(_name, _dtutest), dtutest(*_dtutest)
        { }
      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    enum RCTMuxCtrl
    {
        NONE    = 0,
        INIT    = 1 << 0, // set during first restore
        STORE   = 1 << 1, // store operation required
        RESTORE = 1 << 2, // restore operation required
        WAITING = 1 << 3, // set by the kernel if a signal is required
        SIGNAL  = 1 << 4, // used to signal completion to the kernel
        BLOCK   = 1 << 5, // used to tell the kernel that we can be blocked
    };

    struct ActivateReply
    {
        uint64_t opcode;
        uint64_t ep;
        uint64_t msg_addr;
        uint64_t event;
    } M5_PACKED;

    enum class Algorithm
    {
        SHA1,
        SHA224,
        SHA256,
        SHA384,
        SHA512,
        COUNT
    };

    enum class State
    {
        IDLE,
        FETCH_MSG,
        READ_MSG_ADDR,
        READ_MSG,
        READ_DATA,
        STORE_REPLY,
        SEND_REPLY,
        REPLY_WAIT,
        REPLY_ERROR,
        REPLY_SYSCALL,
        REPLY_FETCH,
        REPLY_READ_ADDR,
        REPLY_ACK,
        ACK_MSG,

        CTX_SAVE,
        CTX_SAVE_DONE,
        CTX_WAIT,
        CTX_CHECK,
        CTX_RESTORE,
    };

    PacketPtr createPacket(Addr paddr, size_t size, MemCmd cmd);

    PacketPtr createDtuRegisterPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    PacketPtr createCmdPacket(uint64_t cmd, uint64_t data, uint64_t size, uint64_t off);

    void freePacket(PacketPtr pkt);

    bool sendPkt(PacketPtr pkt);

    void completeRequest(PacketPtr pkt);

    void recvRetry();

    static Addr getRegAddr(DtuReg reg);

    static Addr getRegAddr(CmdReg reg);

    static Addr getRegAddr(unsigned reg, unsigned epid);

    System *system;

    EventWrapper<DtuAccelHash, &DtuAccelHash::tick> tickEvent;

    CpuPort port;

    DtuAccelHashAlgorithm *algos[5];

    Addr chunkSize;

    bool irqPending;

    State state;

    Algorithm algo;
    Addr msgAddr;
    Addr dataAddr;
    Addr dataOff;
    Addr dataSize;
    Addr remSize;

    size_t replyOffset;
    struct {
        uint64_t count;
        uint8_t bytes[64];
    } M5_ATTR_PACKED reply;

    Addr sysreplyAddr;

    /// Request id for all generated traffic
    MasterID masterId;

    unsigned int id;

    const bool atomic;

    Addr reg_base;

    /// Stores the Packet for later retry
    PacketPtr retryPkt;
};

#endif // __CPU_DTU_ACCEL_HASH_ACCELERATOR_HH__
