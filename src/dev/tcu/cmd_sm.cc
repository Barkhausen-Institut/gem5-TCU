/*
 * Copyright (C) 2021 Nils Asmussen, Barkhausen Institut
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

#include "cmd_sm.hh"

std::string
CommandSM::stateName() const
{
    const char* names[] = { "IDLE", "SEND", "WAIT" };
    return names[static_cast<size_t>(state)];
}

void
CommandSM::executeCommand(PacketPtr cmdPkt)
{
    assert(isIdle());
    assert(!cmd);
    state = CMD_SEND;
    cmd = cmdPkt;
    tick();
}

void
CommandSM::tick()
{
    PacketPtr pkt = nullptr;

    switch (state) {
        case State::CMD_IDLE: {
            exec->commandFinished();
            break;
        }
        case State::CMD_SEND: {
            pkt = cmd;
            break;
        }
        case State::CMD_WAIT: {
            Addr regAddr = TcuIf::getRegAddr(UnprivReg::COMMAND);
            pkt = tcu.createTcuRegPkt(regAddr, 0, MemCmd::ReadReq);
            break;
        }
    }

    if (pkt != nullptr)
        exec->sendMemoryReq(pkt, Cycles(1));
}

void
CommandSM::handleMemResp(PacketPtr pkt)
{
    RequestPtr req = pkt->req;

    Cycles delay(1);
    if (pkt->isError()) {
        warn("%s access failed at %#x\n", pkt->isWrite() ? "Write" : "Read",
            req->getPaddr());
    } else {
        switch (state) {
            case State::CMD_IDLE: {
                assert(false);
                break;
            }
            case State::CMD_SEND: {
                cmd = nullptr;
                state = State::CMD_WAIT;
                break;
            }
            case State::CMD_WAIT: {
                RegFile::reg_t reg = *pkt->getConstPtr<RegFile::reg_t>();
                if ((reg & 0xF) == 0)
                    state = State::CMD_IDLE;
                break;
            }
        }
    }

    tcu.freePacket(pkt);

    // kick things into action again
    exec->scheduleCommand(delay);
}
