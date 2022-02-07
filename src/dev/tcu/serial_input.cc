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

#include "debug/TcuSerialInput.hh"

#include "sim/eventq.hh"
#include "sim/sim_exit.hh"

#include "serial_input.hh"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

TcuSerialInput::DataEvent::DataEvent(TcuSerialInput *s, int fd, int e)
    : PollEvent(fd, e), ser(s)
{
}

void
TcuSerialInput::DataEvent::process(int revent)
{
    // As a consequence of being called from the PollQueue, we might
    // have been called from a different thread. Migrate to "our"
    // thread.
    EventQueue::ScopedMigration migrate(ser->eventQueue());

    ser->data();
}

TcuSerialInput::TcuSerialInput(const Params &p)
    : ClockedObject(p),
      dataEvent(NULL),
      tcuMasterPort(name() + ".tcu_master_port", this),
      tcuSlavePort(name() + ".tcu_slave_port", this),
      tickEvent(this),
      tcu(p.tcu_regfile_base_addr, p.system->getRequestorId(this, name()), p.id),
      cmdSM(tcu, this),
      buffer(),
      pos()
{
    // don't configure stdin if it's no terminal
    if (!isatty(STDIN_FILENO))
        return;

    // make stdin non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    assert(flags != -1);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    // enter raw mode
    struct termios cur;
    tcgetattr(STDIN_FILENO, &old);
    memcpy(&cur, &old, sizeof(old));
    cur.c_lflag &= ~(ICANON | ISIG | ECHO);
    cur.c_cc[VMIN] = 1;
    cur.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &cur);

    // schedule event
    dataEvent = new DataEvent(this, STDIN_FILENO, POLLIN);
    pollQueue.schedule(dataEvent);

    cprintf("Gem5 Terminal started; Quit via Ctrl+]\n");
}

TcuSerialInput::~TcuSerialInput()
{
    if (isatty(STDIN_FILENO)) {
        // restore previous mode
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &old);
    }

    if (dataEvent)
        delete dataEvent;
}

void
TcuSerialInput::init()
{
    tcuSlavePort.sendRangeChange();
}

Port&
TcuSerialInput::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "tcu_master_port")
        return tcuMasterPort;
    else if (if_name == "tcu_slave_port")
        return tcuSlavePort;
    else
        return SimObject::getPort(if_name, idx);
}

bool
TcuSerialInput::TcuMasterPort::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());

    serialInput.cmdSM.handleMemResp(pkt);
    return true;
}

bool
TcuSerialInput::TcuSlavePort::recvTimingReq(PacketPtr pkt)
{
    assert(pkt->needsResponse());
    assert(pkt->isRead());

    pkt->makeResponse();
    memcpy(pkt->getPtr<uint8_t>(), serialInput.buffer, serialInput.pos);
    serialInput.tcuSlavePort.schedTimingResp(pkt, serialInput.clockEdge(Cycles(1)));

    return true;
}

void
TcuSerialInput::TcuSlavePort::recvFunctional(PacketPtr pkt)
{
    panic("not implemented");
}

Tick
TcuSerialInput::TcuSlavePort::recvAtomic(PacketPtr pkt)
{
    panic("not implemented");
}

AddrRangeList
TcuSerialInput::TcuSlavePort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, 0x1000));
    return ranges;
}

void
TcuSerialInput::data()
{
    ssize_t res = ::read(STDIN_FILENO, buffer + pos, sizeof(buffer) - pos);
    panic_if(res < 0, "unable to read from stdin");
    // if we are currently sending the input, ignore further input
    if(pos > 0) {
        DPRINTF(TcuSerialInput,
                "Send in progress; ignoring input of %u bytes\n", res);
        return;
    }

    DPRINTF(TcuSerialInput, "Read %u bytes:\n", res);
    DDUMP(TcuSerialInput, buffer + pos, res);

    pos += res;
    if(pos > 0) {
        if(pos == 1 && buffer[0] == 0x1d)
            exitSimLoop("ctrl+] encountered", 0, curTick(), 0, true);
        else {
            PacketPtr cmdPkt = tcu.createTcuCmdPkt(
                CmdCommand::create(CmdCommand::SEND, EP_INPUT, Tcu::INVALID_EP_ID),
                CmdData::create(0, pos),
                0
            );
            cmdSM.executeCommand(cmdPkt);
        }
    }
}

void
TcuSerialInput::tick()
{
    cmdSM.tick();
}

void
TcuSerialInput::scheduleCommand(Cycles delay)
{
    schedule(tickEvent, clockEdge(delay));
}

void
TcuSerialInput::sendMemoryReq(PacketPtr pkt, Cycles delay)
{
    tcuMasterPort.schedTimingReq(pkt, clockEdge(delay));
}

void
TcuSerialInput::commandFinished()
{
    DPRINTF(TcuSerialInput, "Send finished\n");
    pos = 0;
}
