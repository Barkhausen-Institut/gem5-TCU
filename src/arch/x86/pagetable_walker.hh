/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * Copyright (c) 2020 Barkhausen Institut
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_X86_PAGE_TABLE_WALKER_HH__
#define __ARCH_X86_PAGE_TABLE_WALKER_HH__

#include <vector>

#include "arch/generic/pagetable_walker.hh"
#include "arch/x86/pagetable.hh"
#include "arch/x86/tlb.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "params/X86PagetableWalker.hh"
#include "sim/clocked_object.hh"
#include "sim/faults.hh"
#include "sim/system.hh"

class ThreadContext;

namespace X86ISA
{
    class Walker : public BaseWalker
    {
      protected:
        // State to track each walk of the page table
        class WalkerState : public BaseWalkerState
        {
          friend class Walker;
          private:
            enum TranslateState {
                Idle,
                // Long mode
                LongPML4, LongPDP, LongPD, LongPTE,
                // PAE legacy mode
                PAEPDP, PAEPD, PAEPTE,
                // Non PAE legacy mode with and without PSE
                PSEPD, PD, PTE
            };

          protected:
            TranslateState xlState;
            TranslateState nextXlState;
            int dataSize;
            bool enableNX;
            TlbEntry entry;
          public:
            WalkerState(BaseWalker * _walker,
                        BaseTLB::Translation *_translation,
                        const RequestPtr &_req, bool _isFunctional = false) :
                BaseWalkerState(_walker, _translation, _req, _isFunctional),
                xlState(Idle), nextXlState(Idle), dataSize(0), enableNX(),
                entry()
            {
            }

          protected:
            Walker *ourWalker()
            {
                return static_cast<Walker*>(walker);
            }

            void setupWalk(Addr vaddr) override;
            Fault stepWalk(PacketPtr &write) override;
            void finishFunctional(Addr &addr, unsigned &logBytes) override;
            Fault translateWithTLB(const RequestPtr &req, ThreadContext *tc,
                                   BaseTLB::Translation *translation,
                                   BaseTLB::Mode mode, bool &delayed) override;
            Fault pageFault(bool present);
        };

        // The TLB we're supposed to load.
        TLB * tlb;
        MasterID masterId;

      public:

        void setTLB(TLB * _tlb)
        {
            tlb = _tlb;
        }

        BaseWalkerState *createState(BaseWalker *walker,
                                     BaseTLB::Translation *translation,
                                     const RequestPtr &req,
                                     bool isFunctional) override;

        typedef X86PagetableWalkerParams Params;

        const Params *
        params() const
        {
            return static_cast<const Params *>(_params);
        }

        Walker(const Params *params) :
            BaseWalker(params), tlb(NULL),
            masterId(params->system->getMasterId(this))
        {
        }
    };
}

#endif // __ARCH_X86_PAGE_TABLE_WALKER_HH__
