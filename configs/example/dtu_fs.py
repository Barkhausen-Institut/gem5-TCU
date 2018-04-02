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

import optparse
import sys
import os
import ConfigParser

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../hw/gem5/configs')

from common.FSConfig import *
from common import Simulation
from common import CpuConfig
from common import CacheConfig
from common import MemConfig
from common.Caches import *
from common import Options

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

base_offset                     = 128 * 1024 * 1024
mod_offset                      = base_offset
mod_size                        = 16 * 1024 * 1024
pe_offset                       = mod_offset + mod_size
pe_size                         = 8 * 1024 * 1024

class PcPciHost(GenericPciHost):
    conf_base = 0x30000000
    conf_size = "16MB"

    pci_pio_base = 0

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
    parser.add_option("--cpu-type", type="choice", default="DerivO3CPU",
                      choices=CpuConfig.cpu_names(),
                      help="type of cpu to run with")

    parser.add_option("--isa", type="choice", default="x86_64",
                      choices=['arm', 'x86_64'],
                      help="The ISA to use")

    parser.add_option("-c", "--cmd", default="", type="string",
                      help="comma separated list of binaries")

    parser.add_option("--list-mem-types",
                      action="callback", callback=_listMemTypes,
                     help="List available memory types")
    parser.add_option("--mem-type", type="choice", default="DDR3_1600_8x8",
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

    parser.add_option("--coherent", action="store_true", default=False,
                      help="Whether the caches should be kept coherent")

    parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                      metavar="T",
                      help="Stop after T ticks")

    Options.addFSOptions(parser)

    (options, args) = parser.parse_args()

    if args:
        print "Error: script doesn't take any positional arguments"
        sys.exit(1)

    return options

def getCacheStr(cache):
    return '%d KiB (%d-way assoc, %d cycles)' % (
        cache.size.value / 1024, cache.assoc, cache.tag_latency
    )

def printConfig(pe, dtupos):
    print '      DTU  =bufsz:%d B, blocksz:%d B, count:%d, tlb:%d, walker:%d' % \
        (pe.dtu.buf_size.value, pe.dtu.block_size.value, pe.dtu.buf_count,
         pe.dtu.tlb_entries, 1 if pe.dtu.pt_walker else 0)

    try:
        cc = "coherent" if pe.dtu.coherent else "non-coherent"
        print '      L1i$ =%s' % (getCacheStr(pe.l1icache))
        print '      L1d$ =%s' % (getCacheStr(pe.l1dcache))
        try:
            print '      L2$  =%s' % (getCacheStr(pe.l2cache))
        except:
            pass
        try:
            print '      IO$  =%s' % (getCacheStr(pe.iocache))
        except:
            pass

        dtustr = 'DTU+AT' if pe.dtu.pt_walker else 'DTU'
        str = '      Comp =Core' + ('+AT' if not pe.dtu.pt_walker else '') + ' -> '
        if dtupos == 0:
            str += dtustr + ' -> L1$'
        elif dtupos == 1:
            str += 'L1$ -> ' + dtustr
        else:
            str += 'L1$'
        if hasattr(pe, 'l2cache'):
            str += ' -> L2$'
        if dtupos == 2:
            str += ' -> IO$ -> ' + dtustr
        print str
    except:
        try:
            print '      imem =%d KiB' % (int(pe.proxy.size) / 1024)
            print '      Comp =IDE -> Proxy -> DTU'
        except:
            try:
                print '      imem =%d KiB' % (int(pe.mem_ctrl.range.end + 1) / 1024)
                print '      Comp =DTU -> DRAM'
            except:
                print '      imem =%d KiB' % (int(pe.spm.range.end + 1) / 1024)
                print '      Comp =Core -> DTU -> SPM'

def createPE(noc, options, no, systemType, l1size, l2size, spmsize, dtupos, memPE):
    CPUClass = CpuConfig.get(options.cpu_type)

    # each PE is represented by it's own subsystem
    pe = systemType(mem_mode=CPUClass.memory_mode())
    pe.voltage_domain = VoltageDomain(voltage=options.sys_voltage)
    pe.clk_domain = SrcClockDomain(clock=options.cpu_clock,
                                   voltage_domain=pe.voltage_domain)
    pe.core_id = no

    if not l2size is None:
        pe.xbar = SystemXBar()
    else:
        pe.xbar = L2XBar()
    pe.xbar.point_of_coherency = True

    # we want to use the instructions, not the memory based pseudo operations
    pe.pseudo_mem_ops = False

    pe.dtu = Dtu()
    pe.dtu.core_id = no

    pe.dtu.coherent = options.coherent
    pe.dtu.num_endpoints = 16
    if dtupos > 0:
        pe.dtu.tlb_entries = 32
    else:
        pe.dtu.tlb_entries = 128

    # connection to noc
    pe.dtu.noc_master_port = noc.slave
    pe.dtu.noc_slave_port  = noc.master

    pe.dtu.slave_region = [AddrRange(0, pe.dtu.mmio_region.start - 1)]

    # for some reason, we need to initialize that here explicitly
    pe.dtu.caches = []

    # create caches
    if not l1size is None:
        pe.l1icache = L1_ICache(size=l1size)
        pe.l1icache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
        pe.l1icache.tag_latency = 4
        pe.l1icache.data_latency = 4
        pe.l1icache.response_latency = 4
        pe.dtu.caches.append(pe.l1icache)

        pe.l1dcache = L1_DCache(size=l1size)
        pe.l1dcache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
        pe.l1dcache.tag_latency = 4
        pe.l1dcache.data_latency = 4
        pe.l1dcache.response_latency = 4
        pe.dtu.caches.append(pe.l1dcache)

        if not l2size is None:
            pe.l2cache = L2Cache(size=l2size)
            pe.l2cache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
            pe.l2cache.tag_latency = 12
            pe.l2cache.data_latency = 12
            pe.l2cache.response_latency = 12
            pe.dtu.caches.append(pe.l2cache)

            pe.l2cache.prefetcher = StridePrefetcher(degree = 16)

            # use a crossbar to connect l1icache and l1dcache to l2cache
            pe.tol2bus = L2XBar()
            pe.l2cache.cpu_side = pe.tol2bus.default
            pe.l2cache.mem_side = pe.xbar.slave

            pe.l1icache.mem_side = pe.tol2bus.slave
            pe.l1dcache.mem_side = pe.tol2bus.slave
        else:
            pe.l1dcache.prefetcher = StridePrefetcher(degree = 16)

            pe.l1icache.mem_side = pe.xbar.slave
            pe.l1dcache.mem_side = pe.xbar.slave

        # connect DTU and caches
        if dtupos == 0:
            pe.l1icache.cpu_side = pe.dtu.icache_master_port
            pe.l1dcache.cpu_side = pe.dtu.dcache_master_port
        else:
            pe.iocache = L1_DCache(size='8kB')
            pe.iocache.tag_latency = 4
            pe.iocache.data_latency = 4
            pe.iocache.response_latency = 4
            pe.iocache.cpu_side = pe.dtu.dcache_master_port
            pe.dtu.caches.append(pe.iocache)
            if not l2size is None and dtupos == 1:
                pe.iocache.mem_side = pe.tol2bus.slave
            else:
                pe.iocache.mem_side = pe.xbar.slave

        # the DTU handles LLC misses
        pe.dtu.cache_mem_slave_port = pe.xbar.default

        # don't check whether the kernel is in memory because a PE does not have memory in this
        # case, but just a cache that is connected to a different PE
        pe.kernel_addr_check = False
    elif not spmsize is None:
        pe.spm = Scratchpad(in_addr_map="true")
        pe.spm.cpu_port = pe.xbar.master
        pe.spm.range = spmsize

    if systemType != MemSystem:
        pe.memory_pe = memPE
        pe.memory_offset = pe_offset + (pe_size * no)
        pe.memory_size = pe_size

    if systemType == MemSystem or l1size is None:
        # for memory PEs or PEs with SPM, we do not need a buffer. for the sake of
        # an easy implementation we just make the buffer very large and the block
        # size as well, so that we can read a packet from SPM/DRAM into the buffer
        # and send it from there. Since that costs no simulated time, it is the
        # same as having no buffer.
        pe.dtu.block_size = pe.dtu.max_noc_packet_size
        pe.dtu.buf_size = pe.dtu.max_noc_packet_size

        # disable the TLB and PT walker
        pe.dtu.tlb_entries = 0
        pe.dtu.pt_walker = False
        pe.dtu.cpu_to_cache_latency = 0

        # the DTU sends requests to SPM/mem via xbar
        pe.dtu.dcache_master_port = pe.xbar.slave
        pe.dtu.icache_master_port = pe.xbar.slave

    pe.system_port = pe.xbar.slave

    return pe

def connectToMem(pe, dport, iport=None, l1size=None, dtupos=0):
    if not l1size is None and dtupos > 0:
        if not iport is None:
            pe.l1icache.cpu_side = iport
        pe.l1dcache.cpu_side = dport
    else:
        if not iport is None:
            pe.dtu.icache_slave_port = iport
        pe.dtu.dcache_slave_port = dport

def createCorePE(noc, options, no, cmdline, memPE, l1size=None, l2size=None,
                 dtupos=0, mmu=False, spmsize='8MB'):
    CPUClass = CpuConfig.get(options.cpu_type)

    sysType = M3ArmSystem if options.isa == 'arm' else M3X86System
    con = ArmConnector if options.isa == 'arm' else X86Connector

    # the DTU can't do address translation behind a cache
    assert dtupos == 0 or mmu

    pe = createPE(
        noc=noc, options=options, no=no, systemType=sysType,
        l1size=l1size, l2size=l2size, spmsize=spmsize, dtupos=dtupos,
        memPE=memPE
    )
    pe.dtu.connector = con()
    pe.readfile = "/dev/stdin"

    # connection to the NoC for initialization
    pe.noc_master_port = noc.slave

    pe.cpu = CPUClass()
    pe.cpu.cpu_id = 0

    connectToMem(pe, pe.cpu.dcache_port, pe.cpu.icache_port, l1size, dtupos)

    # cache misses to MMIO region go to DTU
    if not l1size is None and dtupos > 0:
        pe.dtu.dcache_slave_port = pe.xbar.master
        pe.dtu.slave_region = [pe.dtu.mmio_region]

    # disable PT walker if MMU should be used
    if mmu:
        # for the kernel, we disable all translations in the DTU, letting him
        # run with physical addresses.
        # TODO that is actually not necessary. the kernel could have a PF handler
        if no == 0:
            pe.dtu.tlb_entries = 0
        pe.dtu.pt_walker = False

    if "kernel" in cmdline:
        pe.mod_offset = mod_offset
        pe.mod_size = mod_size

    # Command line
    pe.kernel = cmdline.split(' ')[0]
    pe.boot_osflags = cmdline
    print "PE%02d: %s" % (no, cmdline)
    print '      Core =%s %s @ %s' % (type(pe.cpu), options.isa, options.cpu_clock)
    printConfig(pe, dtupos)

    # if specified, let this PE wait for GDB
    if options.pausepe == no:
        print '      waiting for GDB'
        # = 0, because for us, it's always the first context
        pe.rgdb_wait = 0

    print

    # connect the IO space via bridge to the root NoC
    pe.bridge = Bridge(delay='50ns')
    pe.bridge.master = noc.slave
    pe.bridge.slave = pe.xbar.master
    pe.bridge.ranges = \
        [
        AddrRange(IO_address_space_base,
                  interrupts_address_space_base - 1)
        ]

    pe.cpu.createInterruptController()

    if options.isa == 'x86_64':
        pe.cpu.interrupts[0].pio = pe.xbar.master
        pe.cpu.interrupts[0].int_slave = pe.dtu.connector.irq_master_port
        pe.cpu.interrupts[0].int_master = pe.xbar.slave

    if not l2size is None:
        pe.cpu.itb.walker.port = pe.tol2bus.slave
        pe.cpu.dtb.walker.port = pe.tol2bus.slave
    else:
        pe.cpu.itb.walker.port = pe.xbar.slave
        pe.cpu.dtb.walker.port = pe.xbar.slave

    return pe

def createPersistIdePEBase(noc, options, no, memPE, l1size=None, l2size=None,
                           cache_size=None, spmsize=None, img0=None, img1=None):

    PCI_Addressing_Base             = 0x0
    PCI_End                         = 0xFFFFFFFFFFFFFFFF

    # The format of CONFIG_ADDRESS is the following:
    #
    # 0x80000000 | bus << 16 | device << 11 | function <<  8 | offset

    conf_size = "16MB"
    systemType = SpuSystem

    CPUClass = CpuConfig.get(options.cpu_type)

    # each PE is represented by it's own subsystem
    pe = systemType(mem_mode=CPUClass.memory_mode())
    pe.core_id = no

    pe.voltage_domain = VoltageDomain(voltage=options.sys_voltage)
    pe.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                   voltage_domain=pe.voltage_domain)

    # TODO set latencies
    pe.xbar = NoncoherentXBar(forward_latency=0,
                              frontend_latency=0,
                              response_latency=1,
                              width=16)

    pe.pseudo_mem_ops = False

    pe.dtu = Dtu()
    pe.dtu.core_id = no

    pe.dtu.num_endpoints = 16

    pe.dtu.icache_master_port = pe.xbar.slave
    pe.dtu.dcache_master_port = pe.xbar.slave

    pe.dtu.noc_master_port = noc.slave
    pe.dtu.noc_slave_port  = noc.master

    pe.dtu.coherent = options.coherent

    pe.memory_pe = memPE
    pe.memory_offset = pe_offset + (pe_size * no)
    pe.memory_size = pe_size

    # for memory PEs or PEs with SPM, we do not need a buffer. for the sake of an easy implementation
    # we just make the buffer very large and the block size as well, so that we can read a packet
    # from SPM/DRAM into the buffer and send it from there. Since that costs no simulated time,
    # it is the same as having no buffer.
    if systemType == MemSystem or l1size is None:
        pe.dtu.block_size = pe.dtu.max_noc_packet_size
        pe.dtu.buf_size = pe.dtu.max_noc_packet_size
        # disable the TLB
        pe.dtu.tlb_entries = 0
        pe.dtu.cpu_to_cache_latency = 0

    pe.system_port = pe.xbar.slave
    if hasattr(pe, 'noc_master_port'):
        pe.noc_master_port = noc.slave

    #end of create PE

    # create disks
    disks = []
    for img in [img0, img1]:
        if img is not None:
            disk = CowIdeDisk(driveID='master')
            disk.childImage(img)
            disks.append(disk)

    pe.dtu.connector = PCIConnector()

    pe.proxy = BaseProxy.BaseProxy()
    pe.proxy.id = no
    pe.proxy.size = '16kB'

    pe.dtu.connector.base_proxy = pe.proxy

    pe.iobus = IOXBar()
    pe.bridge = Bridge(delay='50ns')

    pe.platform = DtuPlatform()
    pe.platform.intrctrl = IntrControl()
    pe.platform.idedtuproxy = pe.proxy

    pe.proxy.dev_port = pe.bridge.slave
    pe.proxy.dtu_port = pe.xbar.master
    pe.proxy.int_port = pe.dtu.dcache_slave_port

    pe.bridge.master = pe.iobus.slave

    IO_address_space_base = 0x20000000
    pci_config_address_space_base = 0x30000000
    interrupts_address_space_base = 0xa000000000000000

    pe.bridge.ranges = \
    [ # DMA space still missing
        AddrRange(0,
                  Addr.max)
    ]

    pe.proxy.ranges = \
         [
            AddrRange(PCI_Addressing_Base, # Insert proper address space here
                PCI_End)
         ]

    pe.ide_controller = IdeController(disks=disks,
        pci_func=0, pci_dev=0, pci_bus=0)
    pe.ide_controller.BAR0 = 0x1f0
    pe.ide_controller.BAR0LegacyIO = True
    pe.ide_controller.BAR1 = 0x3f4
    pe.ide_controller.BAR1Size = '3B'
    pe.ide_controller.BAR1LegacyIO = True
    pe.ide_controller.BAR2 = 0x170
    pe.ide_controller.BAR2LegacyIO = True
    pe.ide_controller.BAR3 = 0x374
    pe.ide_controller.BAR3Size = '3B'
    pe.ide_controller.BAR3LegacyIO = True
    pe.ide_controller.BAR4 = 1
    #pe.ide_controller.BAR4Size = '16B'
    pe.ide_controller.Command = 1
    pe.ide_controller.io_shift = 0
    pe.ide_controller.InterruptPin = 1
    pe.ide_controller.InterruptLine = 14
    pe.ide_controller.LegacyIOBase = IO_address_space_base

    pe.ide_controller.ctrl_offset = 0
    pe.behind_pci = IsaFake(pio_addr=0x20000cf8, pio_size=8)

    return pe

def createPersistIdePE(noc, options, no, memPE, l1size=None, l2size=None,
                       cache_size=None, spmsize='64kB', img0=None, img1=None):

    pe = createPersistIdePEBase(noc, options, no, memPE, l1size,
        l2size, cache_size, spmsize, img0, img1)

    def attachIO(pe, bus):
        pe.pci_host.pio = bus.default
        pe.ide_controller.pio = bus.master
        pe.ide_controller.dma = bus.slave
        pe.behind_pci.pio = bus.master

    pe.pci_host = PcPciHost()
    pe.pci_host.platform = pe.platform
    # still possible, also less fiddling
    attachIO(pe, pe.iobus)

    print 'pe%02d: %s' % (no, img0)
    printConfig(pe, 0)
    print

    return pe

def createAccelPE(noc, options, no, accel, memPE, l1size=None, l2size=None, spmsize='64kB'):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memPE=memPE,
        dtupos=0
    )
    pe.dtu.connector = DtuAccelConnector()

    if accel == 'indir':
        pe.accel = DtuAccelInDir()
    elif accel == 'fft':
        pe.accel = DtuAccelStream()
        pe.accel.logic = AccelLogic()
        pe.accel.logic.algorithm = 0
        pe.accel.logic.port = pe.xbar.slave
        pe.accel.buf_size = "8kB"
    elif accel == 'toupper':
        pe.accel = DtuAccelStream()
        pe.accel.logic = AccelLogic()
        pe.accel.logic.algorithm = 1
        pe.accel.logic.port = pe.xbar.slave
        pe.accel.buf_size = "8kB"
    else:
        print 'Accelerator "%s" does not exist' % (accel)
        sys.exit(1)
    pe.dtu.connector.accelerator = pe.accel
    pe.accel.id = no;

    connectToMem(pe, pe.accel.port)

    print 'PE%02d: %s accelerator @ %s' % (no, accel, options.cpu_clock)
    printConfig(pe, 0)
    print

    return pe

def createAladdinPE(noc, options, accel, no, memPE, l1size=None, l2size=None, spmsize='64kB'):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memPE=memPE,
        dtupos=0
    )
    pe.dtu.connector = DtuAccelConnector()

    config = ConfigParser.SafeConfigParser()
    config.read("hw/gem5/src/aladdin/integration-test/with-cpu/" + accel + "/gem5.cfg")

    pe.accel = DtuAccelAladdin()
    pe.accel.accel_id = config.getint(accel, "accelerator_id")
    pe.dtu.connector.accelerator = pe.accel
    pe.accel.id = no;

    pe.accel_xbar = L2XBar()
    pe.accel.port = pe.accel_xbar.slave

    pe.accel.hdp = HybridDatapath(
        clk_domain = pe.clk_domain,
        benchName = accel,
        # TODO: Ideally bench_name would change to output_prefix but that's a
        # pretty big breaking change.
        outputPrefix = config.get(accel, "bench_name"),
        traceFileName = config.get(accel, "trace_file_name"),
        configFileName = config.get(accel, "config_file_name"),
        acceleratorName = "%s_datapath" % accel,
        acceleratorId = config.getint(accel, "accelerator_id"),
        cycleTime = 1,
        useDb = False,
        experimentName = config.get(accel, "experiment_name"),
        enableStatsDump = False,
        executeStandalone = False)
    pe.accel.hdp.connector = pe.dtu.connector
    # pe.accel.hdp.cacheLineFlushLatency = config.getint(accel, "cacheline_flush_latency")
    # pe.accel.hdp.cacheLineInvalidateLatency = config.getint(accel, "cacheline_invalidate_latency")
    pe.accel.hdp.dmaSetupOverhead = config.getint(accel, "dma_setup_overhead")
    pe.accel.hdp.maxDmaRequests = config.getint(accel, "max_dma_requests")
    pe.accel.hdp.numDmaChannels = config.getint(accel, "num_dma_channels")
    pe.accel.hdp.dmaChunkSize = config.getint(accel, "dma_chunk_size")
    pe.accel.hdp.pipelinedDma = config.getboolean(accel, "pipelined_dma")
    pe.accel.hdp.ignoreCacheFlush = config.getboolean(accel, "ignore_cache_flush")
    pe.accel.hdp.invalidateOnDmaStore = config.getboolean(accel, "invalidate_on_dma_store")
    pe.accel.hdp.recordMemoryTrace = config.getboolean(accel, "record_memory_trace")
    pe.accel.hdp.enableAcp = False
    pe.accel.hdp.useAcpCache = True
    pe.accel.hdp.useAladdinDebugger = False
    memory_type = config.get(accel, 'memory_type').lower()
    if memory_type == "cache":
        pe.accel.hdp.cacheSize = config.get(accel, "cache_size")
        pe.accel.hdp.cacheBandwidth = config.get(accel, "cache_bandwidth")
        pe.accel.hdp.cacheQueueSize = config.get(accel, "cache_queue_size")
        pe.accel.hdp.cacheAssoc = config.getint(accel, "cache_assoc")
        pe.accel.hdp.cacheHitLatency = config.getint(accel, "cache_hit_latency")
        pe.accel.hdp.cacheLineSize = 64
        pe.accel.hdp.cactiCacheConfig = config.get(accel, "cacti_cache_config")
        pe.accel.hdp.tlbEntries = config.getint(accel, "tlb_entries")
        pe.accel.hdp.tlbAssoc = config.getint(accel, "tlb_assoc")
        pe.accel.hdp.tlbHitLatency = 0
        pe.accel.hdp.tlbMissLatency = 0
        pe.accel.hdp.tlbCactiConfig = config.get(accel, "cacti_tlb_config")
        pe.accel.hdp.tlbPageBytes = config.getint(accel, "tlb_page_size")
        pe.accel.hdp.numOutStandingWalks = config.getint(accel, "tlb_max_outstanding_walks")
        pe.accel.hdp.tlbBandwidth = config.getint(accel, "tlb_bandwidth")
    elif memory_type == "spad" and options.ruby:
        # If the memory_type is spad, Aladdin will initiate a 1-way cache for every
        # datapath, though this cache will not be used in simulation.
        # Since Ruby doesn't support 1-way cache, so set the assoc to 2.
        pe.accel.hdp.cacheAssoc = 2
    pe.accel.hdp.cache_port = pe.accel_xbar.slave

    connectToMem(pe, pe.accel_xbar.default)

    print 'PE%02d: %s accelerator @ %s' % (no, accel[5:], options.cpu_clock)
    printConfig(pe, 0)
    print

    return pe

def createAbortTestPE(noc, options, no, memPE, l1size=None, l2size=None, spmsize='8MB'):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memPE=memPE,
        dtupos=0
    )
    pe.dtu.connector = BaseConnector()

    pe.cpu = DtuAbortTest()
    pe.cpu.id = no;

    connectToMem(pe, pe.cpu.port)

    print 'PE%02d: aborttest core' % (no)
    printConfig(pe, 0)
    print

    return pe

def createMemPE(noc, options, no, size, dram=True, content=None):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=MemSystem,
        l1size=None, l2size=None, spmsize=None, memPE=0,
        dtupos=0
    )
    pe.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                   voltage_domain=pe.voltage_domain)
    pe.dtu.connector = BaseConnector()

    # use many buffers to prevent that this is a bottleneck (this is just a
    # simulation artefact anyway)
    pe.dtu.buf_count = 8

    if dram:
        pe.mem_ctrl = DDR3_1600_8x8()
        pe.mem_ctrl.device_size = size
        pe.mem_ctrl.port = pe.xbar.master
    else:
        pe.mem_ctrl = Scratchpad(in_addr_map="true")
        pe.mem_ctrl.cpu_port = pe.xbar.master
    pe.mem_ctrl.range = MemorySize(size).value

    if not content is None:
        if os.stat(content).st_size > base_offset:
            print 'File "%s" is too large for memory layout (%u vs %u)' \
              % (content, os.stat(content).st_size, base_offset)
            sys.exit(1)
        pe.mem_file = content

    print 'PE%02d: %s' % (no, content)
    printConfig(pe, 0)
    print

    return pe

def createRoot(options):
    root = Root(full_system=True)

    # Create a top-level voltage domain
    root.voltage_domain = VoltageDomain(voltage=options.sys_voltage)

    # Create a source clock for the system and set the clock period
    root.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                     voltage_domain=root.voltage_domain)

    # All PEs are connected to a NoC (Network on Chip). In this case it's just
    # a simple XBar.
    if options.coherent:
        # A dummy system for the CoherentXBar
        root.noc_system = System()

        root.noc = SystemXBar(system=root.noc_system,
                              point_of_coherency=False)

        root.noc_system.system_port = root.noc.slave
    else:
        root.noc = IOXBar()

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

def runSimulation(root, options, pes):
    # determine types of PEs and their internal memory size
    pemems = []
    for pe in pes:
        size = 0
        if hasattr(pe, 'mem_ctrl'):
            size = int(pe.mem_ctrl.range.end + 1)
            assert size % 4096 == 0, "Memory size not page aligned"
            size |= 2   # mem
        else:
            if hasattr(pe, 'spm'):
                size = int(pe.spm.range.end + 1)
                assert size % 4096 == 0, "Memory size not page aligned"
            else:
                size |= 1 # emem
                if not pe.dtu.pt_walker:
                    size |= 1 << 7 # mmu
                else:
                    size |= 2 << 7 # dtuvm

            if hasattr(pe, 'accel'):
                if type(pe.accel).__name__ == 'DtuAccelAladdin':
                    # ALADDIN accelerator
                    if pe.accel.hdp.benchName == 'test_stencil':
                        size |= 7 << 3
                    elif pe.accel.hdp.benchName == 'test_md':
                        size |= 8 << 3
                    elif pe.accel.hdp.benchName == 'test_spmv':
                        size |= 9 << 3
                    elif pe.accel.hdp.benchName == 'test_fft':
                        size |= 10 << 3
                    else:
                        assert(False);
                elif type(pe.accel).__name__ == 'DtuAccelInDir':
                    size |= 4 << 3 # indir accelerator
                elif int(pe.accel.logic.algorithm) == 0:
                    size |= 5 << 3 # fft accelerator
                elif int(pe.accel.logic.algorithm) == 1:
                    size |= 6 << 3 # toupper accelerator
            elif options.isa == 'arm':
                size |= 2 << 3 # arm
            else:
                size |= 1 << 3 # x86

            if hasattr(pe, 'ide_controller'):
                size = int(pe.proxy.size)
                size |= 11 << 3
        pemems.append(size)

    # give that to the PEs
    for pe in pes:
        setattr(root, 'pe%02d' % pe.core_id, pe)
        try:
            pe.pes = pemems
        except:
            pass

    # Instantiate configuration
    m5.instantiate()

    # Simulate until program terminates
    exit_event = m5.simulate(options.maxtick)

    print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
