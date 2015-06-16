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
import os

import m5
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')

import CpuConfig
import MemConfig

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

parser.add_option("-c", "--cmd", default="",
                  help="The binary to run in syscall emulation mode on PE0.")

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

# Set up the system
system = System(mem_mode = CPUClass.memory_mode())

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                   voltage_domain = system.voltage_domain)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
system.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                       voltage_domain = system.cpu_voltage_domain)

# All PEs are connected to a NoC (Network on Chip). In this case it's just
# a simple XBar.
system.noc = NoncoherentXBar(forward_latency  = 0,
                             frontend_latency = 1,
                             response_latency = 1,
                             width = 8,
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
                              frontend_latency = 0,
                              response_latency = 1,
                              width = 8,
                             )

    pe.scratchpad = Scratchpad()
    pe.scratchpad.port = pe.xbar.master

    pe.dtu = Dtu()
    pe.dtu.core_id = i
    pe.dtu.scratchpad = pe.xbar.slave
    pe.dtu.cpu = pe.xbar.master

    pe.dtu.noc_master = system.noc.slave
    pe.dtu.noc_slave  = system.noc.master

    # we only create only CPU as we only have one system port and therfore
    # cannot load programs to multiple memories
    if i == 0:
        pe.cpu = CPUClass(cpu_id = i)
        pe.cpu.clk_domain = system.cpu_clk_domain

        process = LiveProcess()
        process.cwd = os.getcwd()

        process.executable = options.cmd
        system.system_port = pe.xbar.slave

        process.cmd = process.executable

        pe.cpu.workload = process;

        pe.cpu.createInterruptController()
        pe.cpu.connectAllPorts(pe.xbar)

# setup Memory Controllers (from MemConfig.config_mem)

nbr_mem_ctrls = options.mem_channels
intlv_bits = int(math.log(nbr_mem_ctrls, 2))
if 2 ** intlv_bits != nbr_mem_ctrls:
    fatal("Number of memory channels must be a power of 2")

cls = MemConfig.get(options.mem_type)
mem_ctrls = []

# We don't have chaches so we set to fixed 128 Byte granularity
intlv_size = 128

for i in xrange(nbr_mem_ctrls):
    mem_ctrl = MemConfig.create_mem_ctrl(cls,
                                         AddrRange(options.mem_size),
                                         i,
                                         nbr_mem_ctrls,
                                         intlv_bits,
                                         intlv_size)

    # Set the number of ranks based on the command-line
    # options if it was explicitly set
    if issubclass(cls, m5.objects.DRAMCtrl) and \
            options.mem_ranks:
        mem_ctrl.ranks_per_channel = options.mem_ranks

    mem_ctrl.port = system.noc.master

    mem_ctrls.append(mem_ctrl)

system.mem_ctrls = mem_ctrls

root = Root(full_system = False, system = system)

# Instantiate configuration
m5.instantiate()

system.pe0.cpu.workload[0].map(0x1000000, 0x1000000, 1024)

# Simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
