/*
 * Copyright (c) 2015, Christian Menard
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

#ifndef __MEM_DTU_MSG_UNIT_HH__
#define __MEM_DTU_MSG_UNIT_HH__

#include "mem/dtu/dtu.hh"

class MessageUnit
{
  private:

    Dtu &dtu;

  public:

    MessageUnit(Dtu &_dtu) : dtu(_dtu) {}

    /**
     * Start message transmission -> SPM request
     */
    void startTransmission(const Dtu::Command& cmd);

    /**
     * SPM response -> NoC request
     */
    void sendToNoc(const uint8_t* data,
                   Addr messageSize,
                   bool isReply,
                   Tick spmPktHeaderDelay,
                   Tick spmPktPayloadDelay);

    /**
     * Received a message from NoC -> SPM request
     */
    void recvFromNoc(PacketPtr pkt);

    /**
     * Receive message: got response from SPM -> NoC response
     */
    void recvFromNocComplete(PacketPtr pkt, unsigned epId);

    /**
     * Move read pointer forward
     */
    void incrementReadPtr(unsigned epId);

  private:

    bool incrementWritePtr(unsigned epId);

};

#endif
