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

addToPath('../hw/gem5/configs/common')

from FSConfig import *
from Caches import *
import CpuConfig
import MemConfig
import Options

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

# global constants
IO_address_space_base           = 0xff20000000000000
interrupts_address_space_base   = 0xff40000000000000
APIC_range_size                 = 1 << 12;

base_offset                     = 64 * 1024 * 1024
mod_offset                      = base_offset
mod_size                        = 16 * 1024 * 1024
pe_offset                       = mod_offset + mod_size
pe_size                         = 8 * 1024 * 1024

# reads the options and returns them
def getOptions():
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

    parser.add_option("-c", "--cmd", default="", type="string",
                      help="comma separated list of binaries")

    parser.add_option("--list-mem-types",
                      action="callback", callback=_listMemTypes,
                     help="List available memory types")
    parser.add_option("--mem-type", type="choice", default="DDR3_1600_x64",
                      choices=MemConfig.mem_names(),
                      help="type of memory to use")
    parser.add_option("--mem-channels", type="int", default=1,
                      help="number of memory channels")
    parser.add_option("--mem-ranks", type="int", default=None,
                      help="number of memory ranks per channel")

    parser.add_option("--pausepe", default=-1, type="int",
                      help="the PE to pause until GDB connects")

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

    Options.addFSOptions(parser)

    (options, args) = parser.parse_args()

    if args:
        print "Error: script doesn't take any positional arguments"
        sys.exit(1)

    return options

def createPE(root, options, no, mem, l1size, l2size, spmsize, memPE):
    CPUClass = CpuConfig.get(options.cpu_type)

    # each PE is represented by it's own subsystem
    if mem:
        pe = MemSystem(mem_mode=CPUClass.memory_mode())
    else:
        pe = M3X86System(mem_mode=CPUClass.memory_mode())
        pe.core_id = no
    setattr(root, 'pe%02d' % no, pe)

    # TODO set latencies
    pe.xbar = NoncoherentXBar(forward_latency=0,
                              frontend_latency=0,
                              response_latency=1,
                              width=16)
    pe.xbar.clk_domain = root.cpu_clk_domain

    pe.pseudo_mem_ops = False

    pe.dtu = Dtu()
    pe.dtu.core_id = no
    pe.dtu.clk_domain = root.cpu_clk_domain

    pe.dtu.icache_master_port = pe.xbar.slave
    pe.dtu.dcache_master_port = pe.xbar.slave

    pe.dtu.noc_master_port = root.noc.slave
    pe.dtu.noc_slave_port  = root.noc.master

    if not l1size is None:
        pe.dtu.l1cache = L1Cache(size=l1size)
        pe.dtu.l1cache.forward_snoops = False
        pe.dtu.l1cache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
        pe.dtu.l1cache.cpu_side = pe.xbar.master

        if not l2size is None:
            pe.dtu.l2cache = L2Cache(size=l2size)
            pe.dtu.l2cache.forward_snoops = False
            pe.dtu.l2cache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
            pe.dtu.l2cache.cpu_side = pe.dtu.l1cache.mem_side
            pe.dtu.l2cache.mem_side = pe.dtu.cache_mem_slave_port
        else:
            pe.dtu.l1cache.mem_side = pe.dtu.cache_mem_slave_port

        # don't check whether the kernel is in memory because a PE does not have memory in this
        # case, but just a cache that is connected to a different PE
        pe.kernel_addr_check = False
    elif not spmsize is None:
        pe.spm = Scratchpad(in_addr_map="true")
        pe.spm.cpu_port = pe.xbar.master
        pe.spm.range = spmsize

    if not mem:
        pe.memory_pe = memPE
        pe.memory_offset = pe_offset + (pe_size * no)
        pe.memory_size = pe_size
    else:
        pe.dtu.buf_count = 8

    # for memory PEs or PEs with SPM, we do not need a buffer. for the sake of an easy implementation
    # we just make the buffer very large and the block size as well, so that we can read a packet
    # from SPM/DRAM into the buffer and send it from there. Since that costs no simulated time,
    # it is the same as having no buffer.
    if mem or l1size is None:
        pe.dtu.block_size = pe.dtu.max_noc_packet_size
        pe.dtu.buf_size = pe.dtu.max_noc_packet_size
        # disable the TLB
        pe.dtu.tlb_entries = 0

    pe.system_port = pe.xbar.slave
    if not mem:
        pe.noc_master_port = root.noc.slave

    return pe

def createCorePE(root, options, no, cmdline, memPE, l1size=None, l2size=None, spmsize='8MB'):
    CPUClass = CpuConfig.get(options.cpu_type)

    pe = createPE(
        root=root, options=options, no=no, mem=False,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memPE=memPE
    )
    pe.dtu.connector = X86Connector()
    pe.readfile = "/dev/stdin"

    pe.cpu = CPUClass()
    pe.cpu.cpu_id = 0
    pe.cpu.clk_domain = root.cpu_clk_domain

    pe.dtu.icache_slave_port = pe.cpu.icache_port
    pe.dtu.dcache_slave_port = pe.cpu.dcache_port

    if "kernel" in cmdline:
        pe.mod_offset = mod_offset
        pe.mod_size = mod_size

    # Command line
    pe.kernel = cmdline.split(' ')[0]
    pe.boot_osflags = cmdline
    print "PE%02d: %s" % (no, cmdline)
    print '      core   =%s x86' % (options.cpu_type)
    try:
        print '      L1cache=%d KiB' % (pe.dtu.l1cache.size.value / 1024)
        if not l2size is None:
            print '      L2cache=%d KiB' % (pe.dtu.l2cache.size.value / 1024)
    except:
        print '      memsize=%d KiB' % (int(pe.spm.range.end + 1) / 1024)
    print '      bufsize=%d B, blocksize=%d B, count=%d' % \
        (pe.dtu.buf_size.value, pe.dtu.block_size.value, pe.dtu.buf_count)

    # if specified, let this PE wait for GDB
    if options.pausepe == no:
        print '      waiting for GDB'
        # = 0, because for us, it's always the first context
        pe.rgdb_wait = 0

    print

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
    pe.cpu.interrupts.int_slave = pe.dtu.connector.irq_master_port
    pe.cpu.interrupts.int_master = pe.xbar.slave

    pe.cpu.itb.walker.port = pe.xbar.slave
    pe.cpu.dtb.walker.port = pe.xbar.slave

    return pe

def createHashAccelPE(root, options, no, spmsize='64kB'):
    pe = createPE(
        root=root, options=options, no=no, mem=True,
        l1size=None, l2size=None, spmsize=spmsize, memPE=0
    )
    pe.dtu.connector = DtuAccelHashConnector()

    pe.accelhash = DtuAccelHash()
    pe.dtu.connector.accelerator = pe.accelhash
    pe.dtu.dcache_slave_port = pe.accelhash.port
    pe.accelhash.id = no;

    print 'PE%02d: hash accelerator' % (no)
    print '      memsize=%d KiB' % (int(pe.spm.range.end + 1) / 1024)
    print '      bufsize=%d B, blocksize=%d B, count=%d' % \
        (pe.dtu.buf_size.value, pe.dtu.block_size.value, pe.dtu.buf_count)
    print

    return pe

def createMemPE(root, options, no, size, content=None):
    pe = createPE(
        root=root, options=options, no=no, mem=True,
        l1size=None, l2size=None, spmsize=None, memPE=0
    )
    pe.dtu.connector = BaseConnector()

    pe.mem_ctrl = DDR3_1600_x64()
    pe.mem_ctrl.device_size = size
    pe.mem_ctrl.range = MemorySize(size).value
    pe.mem_ctrl.port = pe.xbar.master
    if not content is None:
        if os.stat(content).st_size > base_offset:
            print 'File "%s" is too large for memory layout (%u vs %u)' \
              % (content, os.stat(content).st_size, base_offset)
            sys.exit(1)
        pe.mem_file = content

    print 'PE%02d: %s' % (no, content)
    print '      memsize=%d KiB' % (int(pe.mem_ctrl.range.end + 1) / 1024)
    print '      bufsize=%d B, blocksize=%d B, count=%d' % \
        (pe.dtu.buf_size.value, pe.dtu.block_size.value, pe.dtu.buf_count)
    print

    return pe

def createRoot(options):
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
                               width=12)

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

    return root

def runSimulation(options, pes):
    # determine types of PEs and their internal memory size
    pemems = []
    for pe in pes:
        size = 0
        if hasattr(pe, 'mem_ctrl'):
            size = int(pe.mem_ctrl.device_size)
            assert size % 4096 == 0, "Memory size not page aligned"
            size |= 2   # mem
        else:
            if hasattr(pe, 'spm'):
                size = int(pe.spm.range.end + 1)
                assert size % 4096 == 0, "Memory size not page aligned"
            else:
                size |= 1 # emem

            if hasattr(pe, 'accelhash'):
                size |= 3 << 3 # hash accelerator
            else:
                size |= 1 << 3 # x86
        pemems.append(size)

    # give that to the PEs
    for pe in pes:
        try:
            pe.pes = pemems
        except:
            pass

    # Instantiate configuration
    m5.instantiate()

    # Simulate until program terminates
    exit_event = m5.simulate(options.maxtick)

    print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
