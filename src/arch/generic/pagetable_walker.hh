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

#ifndef __ARCH_GENERIC_WALKER_HH__
#define __ARCH_GENERIC_WALKER_HH__

#include <memory>
#include <vector>

#include "arch/generic/tlb.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "params/BasePagetableWalker.hh"
#include "sim/clocked_object.hh"
#include "sim/faults.hh"
#include "sim/system.hh"

class ThreadContext;

class BaseWalker : public ClockedObject
{
  protected:
    // Port for accessing memory
    class BaseWalkerPort : public MasterPort
    {
      public:
        BaseWalkerPort(const std::string &_name, BaseWalker * _walker) :
              MasterPort(_name, _walker), walker(_walker)
        {}

      protected:
        BaseWalker *walker;

        bool recvTimingResp(PacketPtr pkt);
        void recvReqRetry();
    };

    friend class BaseWalkerPort;
    BaseWalkerPort port;

    // State to track each walk of the page table
    class BaseWalkerState
    {
      friend class BaseWalker;

      protected:
        enum State {
            Ready,
            Waiting,
            Translate,
        };

        BaseWalker *walker;
        ThreadContext *tc;
        RequestPtr req;
        State state;
        State nextState;
        unsigned inflight;
        PacketPtr read;
        std::vector<PacketPtr> writes;
        Fault timingFault;
        BaseTLB::Translation * translation;
        BaseTLB::Mode mode;
        bool functional;
        bool timing;
        bool retrying;
        bool started;
        bool squashed;
      public:
        BaseWalkerState(BaseWalker * _walker,
                        BaseTLB::Translation *_translation,
                        const RequestPtr &_req, bool _isFunctional = false) :
            walker(_walker), req(_req), state(Ready),
            nextState(Ready), inflight(0),
            translation(_translation),
            functional(_isFunctional), timing(false),
            retrying(false), started(false), squashed(false)
        {
        }
        virtual ~BaseWalkerState() {}
        void initState(ThreadContext * _tc, BaseTLB::Mode _mode,
                       bool _isTiming = false);
        Fault startWalk();
        Fault startFunctional(Addr &addr, unsigned &logBytes);
        bool recvPacket(PacketPtr pkt);
        unsigned numInflight() const;
        bool isRetrying();
        bool wasStarted();
        bool isTiming();
        void retry();
        void squash();
        std::string name() const {return walker->name();}

      protected:
        virtual void setupWalk(Addr vaddr) = 0;
        virtual Fault stepWalk(PacketPtr &write) = 0;
        virtual void endWalk();
        virtual void finishFunctional(Addr &addr, unsigned &logBytes) = 0;

        // Performs a TLB lookup for the given translation, assuming that the
        // TLB entry has been created before.
        virtual Fault translateWithTLB(const RequestPtr &req,
                                       ThreadContext *tc,
                                       BaseTLB::Translation *translation,
                                       BaseTLB::Mode mode, bool &delayed) = 0;

      private:
        void sendPackets();
    };

    friend class BaseWalkerState;
    // State for timing and atomic accesses (need multiple per walker in
    // the case of multiple outstanding requests in timing mode)
    std::list<BaseWalkerState *> currStates;
    // State for functional accesses (only need one of these per walker)
    std::unique_ptr<BaseWalkerState> funcState;

    struct WalkerSenderState : public Packet::SenderState
    {
        BaseWalkerState * senderWalk;
        WalkerSenderState(BaseWalkerState * _senderWalk) :
            senderWalk(_senderWalk) {}
    };

  public:
    // Kick off the state machine.
    Fault start(ThreadContext * _tc, BaseTLB::Translation *translation,
            const RequestPtr &req, BaseTLB::Mode mode);
    Fault startFunctional(ThreadContext * _tc, Addr &addr,
            unsigned &logBytes, BaseTLB::Mode mode);
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

  protected:
    System * sys;

    // The number of outstanding walks that can be squashed per cycle.
    unsigned numSquashable;

    // Wrapper for checking for squashes before starting a translation.
    void startWalkWrapper();

    /**
     * Event used to call startWalkWrapper.
     **/
    EventFunctionWrapper startWalkWrapperEvent;

    // Functions for dealing with packets.
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();
    bool sendTiming(BaseWalkerState * sendingState, PacketPtr pkt);

    // Creates the initial walker state for the given translation
    virtual BaseWalkerState *createState(BaseWalker *walker,
                                         BaseTLB::Translation *_translation,
                                         const RequestPtr &req,
                                         bool isFunctional) = 0;

  public:

    typedef BasePagetableWalkerParams Params;

    const Params *
    params() const
    {
        return static_cast<const Params *>(_params);
    }

    BaseWalker(const Params *params) :
        ClockedObject(params), port(name() + ".port", this),
        funcState(), sys(params->system),
        numSquashable(params->num_squash_per_cycle),
        startWalkWrapperEvent([this]{ startWalkWrapper(); }, name())
    {
    }
    virtual ~BaseWalker() {}
};

#endif // __ARCH_GENERIC_ABLE_WALKER_HH__