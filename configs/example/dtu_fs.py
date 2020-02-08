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
from common import CacheConfig
from common import ObjectList
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

base_offset                     = 2048 * 1024 * 1024
mod_offset                      = base_offset
mod_size                        = 32 * 1024 * 1024
pe_offset                       = mod_offset + mod_size
pe_size                         = 16 * 1024 * 1024

class PcPciHost(GenericPciHost):
    conf_base = 0x30000000
    conf_size = "16MB"

    pci_pio_base = 0

# reads the options and returns them
def getOptions():
    parser = optparse.OptionParser()

    parser.add_option("--cpu-type", type="choice", default="DerivO3CPU",
                      choices=ObjectList.cpu_list.get_names(),
                      help="type of cpu to run with")

    parser.add_option("--isa", type="choice", default="x86_64",
                      choices=['arm', 'riscv', 'x86_64'],
                      help="The ISA to use")

    parser.add_option("-c", "--cmd", default="", type="string",
                      help="comma separated list of binaries")

    parser.add_option("--mods", default="", type="string",
                      help="comma separated list of boot modules")

    parser.add_option("--mem-type", type="choice", default="DDR3_1600_8x8",
                      choices=ObjectList.mem_list.get_names(),
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

    options.dot_config = ''
    options.mem_watches = {}

    if args:
        print "Error: script doesn't take any positional arguments"
        sys.exit(1)

    return options

def getCacheStr(cache):
    return '%d KiB (%d-way assoc, %d cycles)' % (
        cache.size.value / 1024, cache.assoc, cache.tag_latency
    )

def printConfig(pe, dtupos):
    print '      DTU  =eps:%d, bufsz:%d B, blocksz:%d B, count:%d, tlb:%d' % \
        (pe.dtu.num_endpoints, pe.dtu.buf_size.value, pe.dtu.block_size.value,
         pe.dtu.buf_count, pe.dtu.tlb_entries)

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

        str = '      Comp =Core -> '
        if dtupos == 0:
            str += 'DTU -> L1$'
        elif dtupos == 1:
            str += 'L1$ -> DTU'
        else:
            str += 'L1$'
        if hasattr(pe, 'l2cache'):
            str += ' -> L2$'
        if dtupos == 2:
            str += ' -> IO$ -> DTU'
        print str
    except:
        try:
            print '      imem =%d KiB' % (int(pe.spm.range.end) / 1024)
            print '      Comp =Core -> DTU -> SPM'
        except:
            pass

def interpose(pe, options, name, port):
    if options.mem_watches.has_key(int(pe.core_id)):
        watch = options.mem_watches[int(pe.core_id)]
        mon = CommMonitor()
        mon.trace = MemWatchProbe(probe_name="PktResponse", ranges=watch)
        mon.slave = port
        setattr(pe, name, mon)
        return mon.master
    return port

def connectDtuToMem(pe, options, l1size, dtupos):
    dport = interpose(pe, options, 'dtudmon', pe.dtu.dcache_master_port)
    if l1size is None or dtupos == 0:
        iport = interpose(pe, options, 'dtuimon', pe.dtu.icache_master_port)

    if not l1size is None:
        if dtupos == 0:
            pe.l1icache.cpu_side = iport
            pe.l1dcache.cpu_side = dport
        else:
            pe.iocache.cpu_side = dport
    else:
        pe.xbar.slave = iport
        pe.xbar.slave = dport

def connectCuToMem(pe, options, dport, iport=None, l1size=None, dtupos=0):
    dport = interpose(pe, options, 'cu_dmon', dport)
    if not iport is None:
        iport = interpose(pe, options, 'cu_imon', iport)

    if not l1size is None and dtupos > 0:
        if not iport is None:
            pe.l1icache.cpu_side = iport
        pe.l1dcache.cpu_side = dport
    else:
        if not iport is None:
            pe.dtu.icache_slave_port = iport
        pe.dtu.dcache_slave_port = dport

def createPE(noc, options, no, systemType, l1size, l2size, spmsize, dtupos, memPE):
    CPUClass = ObjectList.cpu_list.get(options.cpu_type)

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

    pe.dtu = Dtu(max_noc_packet_size='2kB', buf_size='2kB')
    pe.dtu.core_id = no

    pe.dtu.coherent = options.coherent
    pe.dtu.num_endpoints = 192
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

        if dtupos > 0:
            pe.iocache = L1_DCache(size='8kB')
            pe.iocache.tag_latency = 4
            pe.iocache.data_latency = 4
            pe.iocache.response_latency = 4
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

        # disable the TLB
        pe.dtu.tlb_entries = 0
        pe.dtu.cpu_to_cache_latency = 0

    connectDtuToMem(pe, options, l1size, dtupos)

    pe.system_port = pe.xbar.slave

    return pe

def createCorePE(noc, options, no, cmdline, memPE, l1size=None, l2size=None,
                 dtupos=0, spmsize='8MB'):
    CPUClass = ObjectList.cpu_list.get(options.cpu_type)

    if options.isa == 'arm':
        sysType = M3ArmSystem
        con = ArmConnector
    elif options.isa == 'riscv':
        sysType = M3RiscvSystem
        con = RiscvConnector
    else:
        sysType = M3X86System
        con = X86Connector

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

    connectCuToMem(pe, options,
                   pe.cpu.dcache_port,
                   pe.cpu.icache_port,
                   l1size, dtupos)

    # cache misses to MMIO region go to DTU
    if not l1size is None and dtupos > 0:
        pe.dtu.dcache_slave_port = pe.xbar.master
        pe.dtu.slave_region = [pe.dtu.mmio_region]

    if "kernel" in cmdline:
        pe.mod_offset = mod_offset
        pe.mod_size = mod_size
        pe.pe_size = pe_size

    # Command line
    pe.kernel = cmdline.split(' ')[0]
    pe.boot_osflags = cmdline
    print "PE%02d: %s" % (no, cmdline)
    print '      Core =%s %s @ %s' % (type(pe.cpu), options.isa, options.cpu_clock)
    printConfig(pe, dtupos)

    # if specified, let this PE wait for GDB
    if options.pausepe == no:
        print '      waiting for GDB'
        pe.cpu.wait_for_remote_gdb = True

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

    # if not l1size is None:
    #     # connect legacy devices
    #     pe.pc = Pc()
    #     pe.intrctrl = IntrControl()
    #     pe.iobus = IOXBar()
    #     pe.xbar.master = pe.iobus.slave
    #     pe.pc.attachIO(pe.iobus)

    pe.cpu.createThreads()
    pe.cpu.createInterruptController()

    if options.isa == 'x86_64':
        pe.cpu.interrupts[0].pio = pe.xbar.master
        pe.cpu.interrupts[0].int_slave = pe.dtu.connector.irq_master_port
        pe.cpu.interrupts[0].int_master = pe.xbar.slave

    if options.isa != 'riscv':
        if not l2size is None:
            pe.cpu.itb.walker.port = pe.tol2bus.slave
            pe.cpu.dtb.walker.port = pe.tol2bus.slave
        else:
            pe.cpu.itb.walker.port = pe.xbar.slave
            pe.cpu.dtb.walker.port = pe.xbar.slave

    return pe

def createDevicePE(noc, options, no, memPE):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=None, l2size=None, spmsize=None, memPE=memPE,
        dtupos=0
    )
    pe.dtu.connector = BaseConnector()

    # the proxy sits between the device and the DTU to translate interrupts
    # and DMA to the DTU's mechanisms
    pe.proxy = DtuPciProxy()
    pe.proxy.id = no
    pe.proxy.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                         voltage_domain=pe.voltage_domain)

    connectCuToMem(pe, options, pe.proxy.dtu_master_port)
    pe.proxy.dtu_slave_port = pe.xbar.master

    # for the PCI config space
    pe.pci_host = DtuPciHost()
    pe.pci_host.pci_proxy = pe.proxy
    pe.pci_host.conf_base = 0xf000000
    pe.pci_host.conf_size = 0x1000000

    # use bridges to simulate the latency to the I/O devices
    pe.iobus = IOXBar()
    # I/O bridge for requests from the host to the device
    pe.iobridge = Bridge(delay='50ns')
    pe.iobridge.master = pe.iobus.slave
    pe.proxy.pio_port = pe.iobridge.slave
    pe.pci_host.pio = pe.iobus.master

    # DMA bridge for requests from the device to the host
    pe.dmabridge = Bridge(delay='50ns')
    pe.dmabridge.master = pe.proxy.dma_port

    return pe

def createStoragePE(noc, options, no, memPE, img0=None, img1=None):
    pe = createDevicePE(noc, options, no, memPE)

    # create disks
    disks = []
    for img in [img0, img1]:
        if img is not None:
            disk = IdeDisk(driveID='master')
            disk.image = RawDiskImage(image_file = img)
            disks.append(disk)

    pe.idectrl = IdeController(disks=disks,
        pci_func=0, pci_dev=0, pci_bus=0)
    pe.idectrl.BAR0 = 0x1f0
    pe.idectrl.BAR0LegacyIO = True
    pe.idectrl.BAR1 = 0x3f4
    pe.idectrl.BAR1Size = '3B'
    pe.idectrl.BAR1LegacyIO = True
    pe.idectrl.BAR2 = 0x170
    pe.idectrl.BAR2LegacyIO = True
    pe.idectrl.BAR3 = 0x374
    pe.idectrl.BAR3Size = '3B'
    pe.idectrl.BAR3LegacyIO = True
    pe.idectrl.BAR4 = 1
    pe.idectrl.Command = 1
    pe.idectrl.io_shift = 0
    pe.idectrl.InterruptPin = 1
    pe.idectrl.InterruptLine = 14
    pe.idectrl.ctrl_offset = 0

    pe.idectrl.pio = pe.iobus.default
    pe.idectrl.dma = pe.dmabridge.slave

    print 'pe%02d: %s' % (no, img0)
    printConfig(pe, 0)
    print '      Comp =DTU -> Proxy -> IDE'
    print

    return pe

def createEtherPE(noc, options, no, memPE):
    pe = createDevicePE(noc, options, no, memPE)

    pe.nic = IGbE_e1000()
    pe.nic.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                       voltage_domain=pe.voltage_domain)

    pe.nic.host = pe.pci_host
    pe.nic.pci_bus = 0
    pe.nic.pci_dev = 0
    pe.nic.pci_func = 0

    pe.nic.pio = pe.iobus.default
    pe.nic.dma = pe.dmabridge.slave

    print 'PE%02d: IGbE_e1000' % (no)
    printConfig(pe, 0)
    print '      Comp =DTU -> Proxy -> NIC'
    print

    return pe

def linkEtherPEs(ether0, ether1):
    link = EtherLink()
    link.int0 = ether0.nic.interface
    link.int1 = ether1.nic.interface

    ether0.etherlink = link
    ether1.etherlink = link

def createAccelPE(noc, options, no, accel, memPE, l1size=None, l2size=None, spmsize='64kB'):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memPE=memPE,
        dtupos=0
    )
    pe.dtu.connector = DtuAccelConnector()

    if accel == 'indir':
        pe.accel = DtuAccelInDir()
    elif accel == 'copy' or accel == 'rot13':
        algos = {
            'copy'   : 0,
            'rot13'  : 1,
        }
        pe.accel = DtuAccelStream()
        pe.accel.logic = AccelLogic()
        pe.accel.logic.algorithm = algos.get(accel)
        pe.accel.logic.port = pe.xbar.slave
        pe.accel.buf_size = "4kB"
    else:
        print 'Accelerator "%s" does not exist' % (accel)
        sys.exit(1)
    pe.dtu.connector.accelerator = pe.accel
    pe.accel.id = no;

    connectCuToMem(pe, options, pe.accel.port)

    print 'PE%02d: %s accelerator @ %s' % (no, accel, options.cpu_clock)
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

    connectCuToMem(pe, options, pe.cpu.port)

    print 'PE%02d: aborttest core' % (no)
    printConfig(pe, 0)
    print

    return pe

def createMemPE(noc, options, no, size, dram=True, image=None, imageNum=0):
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

    if not image is None:
        if os.stat(image).st_size * imageNum > base_offset:
            print 'File "%s" is too large for memory layout (%u x %u vs %u)' \
              % (image, imageNum, os.stat(image).st_size, base_offset)
            sys.exit(1)
        pe.mem_file = image
        pe.mem_file_num = imageNum

    print 'PE%02d: %s x %d' % (no, image, imageNum)
    printConfig(pe, 0)
    print '      imem =%d KiB' % (int(pe.mem_ctrl.range.end) / 1024)
    name = 'SPM' if type(pe.mem_ctrl).__name__ == 'Scratchpad' else 'DRAM'
    print '      Comp =DTU -> %s' % (name)
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
    root.platform.com_1.device = Terminal()
    root.platform.com_1.pio = root.noc.master

    return root

def runSimulation(root, options, pes):
    # determine types of PEs and their internal memory size
    pemems = []
    for pe in pes:
        size = 0
        if hasattr(pe, 'mem_ctrl'):
            size = int(pe.mem_ctrl.range.end)
            assert size % 4096 == 0, "Memory size not page aligned"
            size |= 2   # mem
        else:
            if hasattr(pe, 'spm'):
                size = int(pe.spm.range.end)
                assert size % 4096 == 0, "Memory size not page aligned"
            else:
                size |= 1 # emem

            if hasattr(pe, 'accel'):
                if type(pe.accel).__name__ == 'DtuAccelInDir':
                    size |= 4 << 3 # indir accelerator
                elif int(pe.accel.logic.algorithm) == 0:
                    size |= 5 << 3 # copy accelerator
                elif int(pe.accel.logic.algorithm) == 1:
                    size |= 6 << 3 # rot13 accelerator
            elif options.isa == 'arm':
                size |= 2 << 3 # arm
            elif options.isa == 'riscv':
                size |= 3 << 3 # riscv
            else:
                size |= 1 << 3 # x86

            if hasattr(pe, 'idectrl'):
                size = 7 << 3
            elif hasattr(pe, 'nic'):
                size = 8 << 3
        pemems.append(size)

    # give that to the PEs
    for pe in pes:
        setattr(root, 'pe%02d' % pe.core_id, pe)
        try:
            pe.mods = options.mods
            pe.pes = pemems
        except:
            pass

    # Instantiate configuration
    m5.instantiate()

    # Simulate until program terminates
    exit_event = m5.simulate(options.maxtick)

    print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
