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

import math
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
parser.add_option("--num-pes", type="int", default=2,
                  help = "Number of PEs (processing elements) in the system"
                  "[default:%default]")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# Start by parsing the command line options and do some basic sanity
# checking

if not options.num_pes > 0:
    print "Error: Must have at least one PE"
    sys.exit(1)

# Set up the system
system = System()

system.voltage_domain = VoltageDomain(voltage = '1V')

system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                        voltage_domain = system.voltage_domain)

# All PEs are connected to a NoC (Network on Chip). In this case it's just
# a simple XBar.
system.noc = NoncoherentXBar(forward_latency  = 0,
                             frontend_latency = 0,
                             response_latency = 0,
                             width = 16, # default 128bit TODO is this a good value??
                             )

# A PE (processing element) consists of a CPU, a Scratchpad-Memory, and a DTU.
# These elements are connected via a simple non-coherent XBar. The DTU ports
# mem_side_slave and mem_side_master are the PE's interface to the outside
# world.
for i in range(0, options.num_pes):

    # each PE is represented by it's own subsystem
    pe = SubSystem()
    setattr(system, 'pe%d' % i, pe)

    # TODO set latencies
    pe.xbar = NoncoherentXBar(forward_latency  = 0,
                              frontend_latency = 1,
                              response_latency = 1,
                              width = 16, # default 128bit TODO is this a good value??
                             )

    pe.cpu = DtuTest()
    pe.cpu.port = pe.xbar.slave

    pe.scratchpad = Scratchpad()
    pe.scratchpad.port = pe.xbar.master

    pe.dtu = Dtu()
    pe.dtu.dtu_addr = i
    pe.dtu.dtu_addr_bits = math.ceil(math.log(options.num_pes,2))
    pe.dtu.scratchpad = pe.xbar.slave
    pe.dtu.cpu = pe.xbar.master

    pe.dtu.master = system.noc.slave
    pe.dtu.slave  = system.noc.master

#system.badaddr1 = IsaFake(pio_addr=0x10000, pio_size=0x0FFEFFFF)
#system.badaddr1.warn_access="warn"
#system.badaddr1.pio = system.xbar.master

#system.badaddr2 = IsaFake(pio_addr=0x10001000, pio_size=0x0FFFFFFFFEFFFEFFF)
#system.badaddr2.warn_access="warn"
#system.badaddr2.pio = system.xbar.master

system.system_port = system.noc.slave

root = Root(full_system = False, system = system)

# Instantiate configuration
m5.instantiate()

# Simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
