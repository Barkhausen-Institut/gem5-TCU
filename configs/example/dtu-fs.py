# Copyright (c) 2015 Christian Menard
# Copyright (c) 2015 Nils Asmussen
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
import os

import m5
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')

from FSConfig import *
import CpuConfig
import MemConfig
import Options

def _listCpuTypes(option, opt, value, parser):
    CpuConfig.print_cpu_list()
    sys.exit(0)

def _listMemTypes(option, opt, value, parser):
    MemConfig.print_mem_list()
    sys.exit(0)

parser = optparse.OptionParser()

parser.add_option("--list-cpu-types",
                  action="callback", callback=_listCpuTypes,
                  help="List available CPU types")
parser.add_option("--cpu-type", type="choice", default="atomic",
                  choices=CpuConfig.cpu_names(),
                  help = "type of cpu to run with")

parser.add_option("-c", "--cmd", default="", type="string",
                  help="comma separated list of binaries")
parser.add_option("--init_mem", default="", type="string",
                  help="file to load into the memory-PE")

parser.add_option("--list-mem-types",
                  action="callback", callback=_listMemTypes,
                 help="List available memory types")
parser.add_option("--mem-type", type="choice", default="DDR3_1600_x64",
                  choices=MemConfig.mem_names(),
                  help = "type of memory to use")
parser.add_option("--mem-size", action="store", type="string",
                  default="512MB",
                  help="Specify the physical memory size (single memory)")
parser.add_option("--mem-channels", type="int", default=1,
                  help = "number of memory channels")
parser.add_option("--mem-ranks", type="int", default=None,
                  help = "number of memory ranks per channel")

parser.add_option("--watch-pe", type="int", default=-1,
                  help = "the PE number for memory watching")
parser.add_option("--watch-start", type="int", default=0,
                  help = "The start address of the address range to watch")
parser.add_option("--watch-end", type="int", default=0,
                  help = "The end address of the address range to watch (exclusive")

parser.add_option("--sys-voltage", action="store", type="string",
                  default='1.0V',
                  help = """Top-level voltage for blocks running at system
                  power supply""")
parser.add_option("--sys-clock", action="store", type="string",
                  default='1GHz',
                  help = """Top-level clock for blocks running at system
                  speed""")
parser.add_option("--cpu-clock", action="store", type="string",
                  default='2GHz',
                  help="Clock for blocks running at CPU speed")

parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                  metavar="T",
                  help="Stop after T ticks")
parser.add_option("-n", "--num-pes", type="int", default=2,
                  help = "Number of PEs (processing elements) in the system"
                  "[default:%default]")

Options.addFSOptions(parser)

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# Start by parsing the command line options and do some basic sanity
# checking

if not options.num_pes > 0:
    print "Error: Must have at least one PE"
    sys.exit(1)

CPUClass = CpuConfig.get(options.cpu_type)

if CPUClass.require_caches():
    fatal("We don't want caches but %s must be used with caches" % options.cpu_type)

# Each PE is represented as an instance of System. Wherea each PE has a CPU,
# a Scratchpad, and a DTU connected via a crossbar. The PEs are connected via
# a global crossbar.

###############################################################################
# root                                                                        #
#                                                                             #
# |-----------------|    |-----------------|    |-----------------|           #
# | pe0             |    | pe1             |    | pe2             |           #
# |         cpu     |    |         cpu     |    |         cpu     |           #
# |          |      |    |          |      |    |          |      |           #
# |        xbar     |    |        xbar     |    |        xbar     |           #
# |       /    \    |    |       /    \    |    |       /    \    |           #
# | scratchpad--dtu |    | scratchpad--dtu |    | scratchpad--dtu |           #
# |              |  |    |              |  |    |              |  |           #
# |--------------+--|    |--------------+--|    |--------------+--|           #
#                |                      |                      |              #
#                |                      |                      |              #
#          noc --O----------------------O----------------------O----          #
#                                                                             #
###############################################################################

def createPE(no, mem=False):
    # each PE is represented by it's own subsystem
    if mem:
        pe = System(mem_mode = CPUClass.memory_mode())
    else:
        pe = M3X86System(mem_mode = CPUClass.memory_mode())
    setattr(root, 'pe%d' % no, pe)

    # TODO set latencies
    pe.xbar = NoncoherentXBar(forward_latency  = 0,
                              frontend_latency = 0,
                              response_latency = 1,
                              width = 8)

    pe.scratchpad = Scratchpad(in_addr_map = "true")
    pe.scratchpad.cpu_port = pe.xbar.master

    if options.watch_pe == no:
        print "PE%u: watching memory %#x..%#x" % (no, options.watch_start, options.watch_end)
        pe.scratchpad.watch_range_start = options.watch_start
        pe.scratchpad.watch_range_end = options.watch_end

    pe.dtu = Dtu()
    pe.dtu.core_id = no
    pe.dtu.spm_master_port = pe.scratchpad.dtu_port
    pe.dtu.cpu_slave_port = pe.xbar.master

    pe.dtu.noc_master_port = root.noc.slave
    pe.dtu.noc_slave_port  = root.noc.master

    pe.system_port = pe.xbar.slave

    return pe


# Set up the system
root = Root(full_system = True)

# Create a top-level voltage domain
root.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

# Create a source clock for the system and set the clock period
root.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                 voltage_domain = root.voltage_domain)

# Create a CPU voltage domain
root.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
root.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                     voltage_domain = root.cpu_voltage_domain)

# All PEs are connected to a NoC (Network on Chip). In this case it's just
# a simple XBar.
root.noc = NoncoherentXBar(forward_latency  = 0,
                           frontend_latency = 1,
                           response_latency = 1,
                           width = 8,
                          )

IO_address_space_base = 0x8000000000000000
interrupts_address_space_base = 0xa000000000000000
APIC_range_size = 1 << 12;

# create a dummy platform and system for the UART
root.platform = IOPlatform()
root.platform.system = System()
root.platform.system.system_port = root.noc.slave
root.platform.intrctrl = IntrControl()

# UART and terminal
root.platform.com_1 = Uart8250()
root.platform.com_1.pio_addr = IO_address_space_base + 0x3f8
root.platform.com_1.terminal = Terminal()
root.platform.com_1.pio = root.noc.master

cmd_list = options.cmd.split(",")
# allow an ',' at the end
if cmd_list[len(cmd_list) - 1] == '':
  cmd_list.pop()

# A PE (processing element) consists of a CPU, a Scratchpad-Memory, and a DTU.
# These elements are connected via a simple non-coherent XBar. The DTU ports
# mem_side_slave and mem_side_master are the PE's interface to the outside
# world.

# currently, there is just one memory-PE
for i in range(0, len(cmd_list)):
    pe = createPE(i)
    pe.readfile = "/dev/stdin"

    # for now, a bit more to be able to put every application at a different address
    pe.scratchpad.range = 4 * 1024 * 1024

    pe.cpu = CPUClass()
    pe.cpu.cpu_id = 0
    pe.cpu.clk_domain = root.cpu_clk_domain

    # Command line
    pe.kernel = cmd_list[i].split(' ')[0]
    pe.boot_osflags = cmd_list[i]
    pe.dtu.use_ptable = 'false'
    print 'PE%d: memsize=%d KiB' % (i, int(pe.scratchpad.range.end) / 1024)
    print "PE%d: cmdline=%s" % (i, cmd_list[i])

    # connect the IO space via bridge to the root NoC
    pe.bridge = Bridge(delay='50ns')
    pe.bridge.master = root.noc.slave
    pe.bridge.slave = pe.xbar.master
    pe.bridge.ranges = \
        [
        AddrRange(IO_address_space_base,
                  interrupts_address_space_base - 1)
        ]

    pe.cpu.createInterruptController()
    pe.cpu.connectAllPorts(pe.xbar)

pe = createPE(options.num_pes, mem=True)
pe.scratchpad.range = MemorySize(options.mem_size).value
pe.scratchpad.init_file = options.init_mem
print 'PE%d: memsize=%d KiB' % (options.num_pes, int(pe.scratchpad.range.end) / 1024)
print 'PE%d: content=%s' % (options.num_pes, options.init_mem)

# Instantiate configuration
m5.instantiate()

# Simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
