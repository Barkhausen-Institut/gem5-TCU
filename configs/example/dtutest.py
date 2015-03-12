# Copyright (c) 2015 Christian Menard
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# The views and conclusions contained in the software and documentation are those
# of the authors and should not be interpreted as representing official policies,
# either expressed or implied, of the FreeBSD Project.

import optparse

import m5
from m5.objects import *

parser = optparse.OptionParser()

parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                  metavar="T",
                  help="Stop after T ticks")
parser.add_option("--sys-clock", action="store", type="string",
                  default='1GHz',
                  help = """Top-level clock for blocks running at system
                  speed""")

(options, args) = parser.parse_args()

if args:
     print "Error: script doesn't take any positional arguments"
     sys.exit(1)

# Make a prototype for the tester to be used throughout
proto_tester = DtuTest()

# Set up the system
system = System()

system.voltage_domain = VoltageDomain(voltage = '1V')

system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                        voltage_domain = system.voltage_domain)


# TODO set latencies
system.xbar = NoncoherentXBar(forward_latency  = 0,
                              frontend_latency = 1,
                              response_latency = 1,
                              width = 16, # default 128bit TODO is this a good value??
                              )

system.cpu = DtuTest()
system.cpu.port = system.xbar.slave

system.badaddr = IsaFake(pio_addr=0x10000, pio_size=0xFFFFFFFFFFFEFFFF)
system.badaddr.warn_access="warn"
system.badaddr.pio = system.xbar.master

system.scratchpad = Scratchpad()
system.scratchpad.port = system.xbar.master

system.system_port = system.xbar.slave

root = Root(full_system = False, system = system)

# Instantiate configuration
m5.instantiate()

# Simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
