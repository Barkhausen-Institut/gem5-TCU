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
from Caches import *
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
                  help="type of cpu to run with")

parser.add_option("--caches", action="store_true",
                  help="use caches in the PEs")
parser.add_option("--l2caches", action="store_true",
                  help="use L2 caches in the PEs")

parser.add_option("-c", "--cmd", default="", type="string",
                  help="comma separated list of binaries")
parser.add_option("--init_mem", default="", type="string",
                  help="file to load into the memory-PE")
parser.add_option("--debug", default="",
                  help="the binary to debug")

parser.add_option("--list-mem-types",
                  action="callback", callback=_listMemTypes,
                 help="List available memory types")
parser.add_option("--mem-type", type="choice", default="DDR3_1600_x64",
                  choices=MemConfig.mem_names(),
                  help="type of memory to use")
parser.add_option("--mem-size", action="store", type="string",
                  default="512MB",
                  help="Specify the physical memory size (single memory)")
parser.add_option("--mem-channels", type="int", default=1,
                  help="number of memory channels")
parser.add_option("--mem-ranks", type="int", default=None,
                  help="number of memory ranks per channel")

parser.add_option("--watch-pe", type="int", default=-1,
                  help="the PE number for memory watching")
parser.add_option("--watch-start", type="int", default=0,
                  help="The start address of the address range to watch")
parser.add_option("--watch-end", type="int", default=0,
                  help="The end address of the address range to watch (exclusive")

parser.add_option("--sys-voltage", action="store", type="string",
                  default='1.0V',
                  help="""Top-level voltage for blocks running at system
                  power supply""")
parser.add_option("--sys-clock", action="store", type="string",
                  default='1GHz',
                  help="""Top-level clock for blocks running at system
                  speed""")
parser.add_option("--cpu-clock", action="store", type="string",
                  default='2GHz',
                  help="Clock for blocks running at CPU speed")

parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                  metavar="T",
                  help="Stop after T ticks")
parser.add_option("-n", "--num-pes", type="int", default=2,
                  help="Number of PEs (processing elements) in the system"
                  "[default:%default]")

Options.addFSOptions(parser)

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# Start by parsing the command line options and do some basic sanity
# checking

IO_address_space_base         = 0xff20000000000000
interrupts_address_space_base = 0xff40000000000000
APIC_range_size = 1 << 12;

base_offset = 32 * 1024 * 1024
mod_offset = base_offset
mod_size = 4 * 1024 * 1024
pe_offset = mod_offset + mod_size
pe_size = 8 * 1024 * 1024

if not options.num_pes > 0:
    print "Error: must have at least one PE"
    sys.exit(1)

CPUClass = CpuConfig.get(options.cpu_type)

# Each PE is represented as an instance of System. Whereas each PE has a CPU,
# a Scratchpad, and a DTU. Because it seems that the gem5 crossbar is not able
# to handle requests from the icache/dcache ports of the CPU if using the O3 model,
# we're connecting the icache/dcache ports to the DTU. The DTU forwards the request
# either to its own register file or the scratchpad, depending on the address range.
# The PEs are connected via a global crossbar.

###############################################################################
# root                                                                        #
#                                                                             #
# |-----------------|    |-----------------|    |-----------------|           #
# | pe0             |    | pe1             |    | pe2             |           #
# |         cpu     |    |         cpu     |    |         cpu     |           #
# |          ||     |    |          ||     |    |          ||     |           #
# |   regs--dtu--+  |    |   regs--dtu--+  |    |   regs--dtu--+  |           #
# |          ||  |  |    |          ||  |  |    |          ||  |  |           #
# |   SPM/cache  |  |    |   SPM/cache  |  |    |   SPM/cache  |  |           #
# |              |  |    |              |  |    |              |  |           #
# |--------------+--|    |--------------+--|    |--------------+--|           #
#                |                      |                      |              #
#                |                      |                      |              #
#          noc --O----------------------O----------------------O----          #
#                                                                             #
###############################################################################

def createPE(no, mem=False, cache=True, l2cache=True, memPE=0):
    # each PE is represented by it's own subsystem
    if mem:
        pe = MemSystem(mem_mode=CPUClass.memory_mode())
    else:
        pe = M3X86System(mem_mode=CPUClass.memory_mode())
        pe.core_id = no
    setattr(root, 'pe%d' % no, pe)

    # TODO set latencies
    pe.xbar = NoncoherentXBar(forward_latency=0,
                              frontend_latency=0,
                              response_latency=1,
                              width=16)

    pe.dtu = Dtu()
    pe.dtu.core_id = no

    pe.dtu.icache_master_port = pe.xbar.slave
    pe.dtu.dcache_master_port = pe.xbar.slave

    pe.dtu.noc_master_port = root.noc.slave
    pe.dtu.noc_slave_port  = root.noc.master

    if not mem:
        if cache:
            pe.dtu.l1cache = L1Cache(size='64kB')
            pe.dtu.l1cache.forward_snoops = False
            pe.dtu.l1cache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
            pe.dtu.l1cache.cpu_side = pe.xbar.master

            if l2cache:
                pe.dtu.l2cache = L2Cache(size='256kB')
                pe.dtu.l2cache.forward_snoops = False
                pe.dtu.l2cache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
                pe.dtu.l2cache.cpu_side = pe.dtu.l1cache.mem_side
                pe.dtu.l2cache.mem_side = pe.dtu.cache_mem_slave_port
            else:
                pe.dtu.l1cache.mem_side = pe.dtu.cache_mem_slave_port

            # don't check whether the kernel is in memory because a PE does not have memory in this
            # case, but just a cache that is connected to a different PE
            pe.kernel_addr_check = False
        else:
            pe.cachespm = Scratchpad(in_addr_map="true")
            pe.cachespm.cpu_port = pe.xbar.master
            pe.cachespm.range = "8MB"

        pe.memory_pe = memPE
        pe.memory_offset = pe_offset + (pe_size * no)
        pe.memory_size = pe_size
        if no == 0:
            pe.mod_offset = mod_offset

    # for memory PEs or PEs with SPM, we do not need a buffer. for the sake of an easy implementation
    # we just make the buffer very large and the block size as well, so that we can read a packet
    # from SPM/DRAM into the buffer and send it from there. Since that costs no simulated time,
    # it is the same as having no buffer.
    if mem or not cache:
        pe.dtu.block_size = pe.dtu.max_noc_packet_size
        pe.dtu.buf_size = pe.dtu.max_noc_packet_size
        # disable the TLB
        pe.dtu.tlb_entries = 0

    if options.watch_pe == no:
        print "PE%u: watching memory %#x..%#x" % (no, options.watch_start, options.watch_end)
        pe.dtu.watch_range_start = options.watch_start
        pe.dtu.watch_range_end = options.watch_end

    pe.system_port = pe.xbar.slave

    return pe

def createCorePE(no, cache, l2cache, memPE):
    pe = createPE(no=no, mem=False, cache=cache, l2cache=l2cache, memPE=memPE)
    pe.readfile = "/dev/stdin"

    pe.cpu = CPUClass()
    pe.cpu.cpu_id = 0
    pe.cpu.clk_domain = root.cpu_clk_domain

    pe.dtu.icache_slave_port = pe.cpu.icache_port
    pe.dtu.dcache_slave_port = pe.cpu.dcache_port

    # Command line
    pe.kernel = cmd_list[no].split(' ')[0]
    pe.boot_osflags = cmd_list[no]
    print "PE%d: %s" % (no, cmd_list[no])
    print '     core   =%s x86' % (options.cpu_type)
    try:
        print '     L1cache=%d KiB' % (pe.dtu.l1cache.size.value / 1024)
        if l2cache:
            print '     L2cache=%d KiB' % (pe.dtu.l2cache.size.value / 1024)
    except:
        print '     memsize=%d KiB' % (int(pe.cachespm.range.end + 1) / 1024)
    print '     bufsize=%d KiB, blocksize=%d B, count=%d' % \
        (pe.dtu.buf_size.value / 1024, pe.dtu.block_size.value, pe.dtu.buf_count)
    print

    # if specified, let this PE wait for GDB
    if options.debug != "" and options.debug in pe.kernel:
        # = 0, because for us, it's always the first context
        pe.rgdb_wait = 0

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

    pe.cpu.interrupts.pio = pe.xbar.master
    pe.cpu.interrupts.int_slave = pe.dtu.irq_master_port
    pe.cpu.interrupts.int_master = pe.xbar.slave

    pe.cpu.itb.walker.port = pe.xbar.slave
    pe.cpu.dtb.walker.port = pe.xbar.slave

def createMemPE(no, size, content=None):
    pe = createPE(no, mem=True)
    pe.mem_ctrl = DDR3_1600_x64()
    pe.mem_ctrl.device_size = size
    pe.mem_ctrl.range = MemorySize(size).value
    pe.mem_ctrl.port = pe.xbar.master
    if not content is None:
        pe.mem_file = content
    print 'PE%d: %s' % (no, content)
    print '     memsize=%d KiB' % (int(pe.mem_ctrl.range.end + 1) / 1024)
    print '     bufsize=%d KiB, blocksize=%d B, count=%d' % \
        (pe.dtu.buf_size.value / 1024, pe.dtu.block_size.value, pe.dtu.buf_count)
    print


# Set up the system
root = Root(full_system=True)

# Create a top-level voltage domain
root.voltage_domain = VoltageDomain(voltage=options.sys_voltage)

# Create a source clock for the system and set the clock period
root.clk_domain = SrcClockDomain(clock= options.sys_clock,
                                 voltage_domain=root.voltage_domain)

# Create a CPU voltage domain
root.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
root.cpu_clk_domain = SrcClockDomain(clock=options.cpu_clock,
                                     voltage_domain=root.cpu_voltage_domain)

# All PEs are connected to a NoC (Network on Chip). In this case it's just
# a simple XBar.
root.noc = NoncoherentXBar(forward_latency=0,
                           frontend_latency=1,
                           response_latency=1,
                           width=8)

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

# create the core PEs
for i in range(0, min(options.num_pes, len(cmd_list))):
    createCorePE(no=i,
                 cache=options.caches,
                 l2cache=options.l2caches,
                 memPE=options.num_pes)

# create the memory PEs
createMemPE(no=options.num_pes,
            size=options.mem_size,
            content=options.init_mem)

# Instantiate configuration
m5.instantiate()

# Simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
