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

#ifndef __MEM_DTU_PT_WALKER_HH__
#define __MEM_DTU_PT_WALKER_HH__

#include "base/types.hh"
#include "base/bitunion.hh"
#include "sim/eventq.hh"
#include "mem/packet.hh"
#include "mem/dtu/noc_addr.hh"
#include "mem/dtu/tlb.hh"

#include <list>

class Dtu;

class PtUnit
{
  private:

    struct TranslateEvent;

  public:

    struct Translation
    {
        friend class PtUnit;

        Translation() : _event()
        {}
        virtual ~Translation()
        {}

        virtual bool causePagefault() { return true; }

        virtual void finished(bool success, const NocAddr &phys) = 0;

      private:

        TranslateEvent *_event;
    };

    BitUnion64(PageTableEntry)
        Bitfield<63, DtuTlb::PAGE_BITS> base;
        Bitfield<4,0> lixwr;
        Bitfield<4> l;
        Bitfield<3> i;
        Bitfield<2> x;
        Bitfield<1> w;
        Bitfield<0> r;
    EndBitUnion(PageTableEntry)

    struct PagefaultMessage
    {
        enum
        {
            OPCODE_PF = 0,
        };

        // have to be words
        uint64_t opcode;
        uint64_t virt;
        uint64_t access;
    } M5_ATTR_PACKED;

  private:

    struct TranslateEvent : public Event
    {
        PtUnit& unit;

        uint64_t id;
        Cycles startCycle;
        Cycles pfStartCycle;
        int level;
        Addr virt;
        Addr ptAddr;
        uint access;
        Translation* trans;
        bool forceWalk;

        TranslateEvent(PtUnit& _unit)
            : unit(_unit), id(nextId++), startCycle(), pfStartCycle(),
              level(), virt(), ptAddr(), access(), trans(), forceWalk()
        {}

        void process() override;

        void recvFromMem(PacketPtr pkt);

        void requestPTE();

        void finish(bool success, const NocAddr &addr);

        const char* description() const override { return "TranslateEvent"; }

        const std::string name() const override;

        static uint64_t nextId;
    };

  public:

    PtUnit(Dtu& _dtu);

    void regStats();

    bool translateFunctional(Addr virt, uint access, NocAddr *phys);

    void startTranslate(Addr virt, uint access, Translation *trans);

    void abortTranslate(Translation *trans);

    void abortAll();

    void recvFromMem(Addr id, PacketPtr pkt)
    {
        TranslateEvent *ev = getEvent(id);
        if (ev)
            ev->recvFromMem(pkt);
    }

    void sendingPfFailed(PacketPtr pkt, int error);

    void finishPagefault(PacketPtr pkt);

  private:

    const char *describeAccess(uint access);

    void mkTlbEntry(Addr virt, NocAddr phys, uint flags);

    TranslateEvent *getEvent(uint64_t id);
    TranslateEvent *getPfEvent(uint64_t id);

    void nextPagefault(TranslateEvent *ev);

    PacketPtr createPacket(Addr virt, Addr ptAddr, int level);

    bool canSendPF() const;

    bool sendPagefaultMsg(TranslateEvent *ev, Addr virt, uint access);

    bool finishTranslate(PacketPtr pkt,
                         Addr virt,
                         int  level,
                         uint *access,
                         Addr *phys);

    Dtu& dtu;

    std::list<TranslateEvent*> translations;
    std::list<TranslateEvent*> pfqueue;

    Stats::Histogram walks;
    Stats::Histogram pagefaults;
    Stats::Scalar unresolved;
    Stats::Scalar delays;

};

#endif
