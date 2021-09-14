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

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../platform/gem5/configs')

from common.FSConfig import *
from common import Simulation
from common import CacheConfig
from common import ObjectList
from common.Caches import *
from common import Options

# Each PE is represented as an instance of System. Whereas each PE has a CPU,
# a Scratchpad, and a TCU. Because it seems that the gem5 crossbar is not able
# to handle requests from the icache/dcache ports of the CPU if using the O3 model,
# we're connecting the icache/dcache ports to the TCU. The TCU forwards the request
# either to its own register file or the scratchpad, depending on the address range.
# The PEs are connected via a global crossbar.

###############################################################################
# root                                                                        #
#                                                                             #
# |-----------------|    |-----------------|    |-----------------|           #
# | pe0             |    | pe1             |    | pe2             |           #
# |         cpu     |    |         cpu     |    |         cpu     |           #
# |          ||     |    |          ||     |    |          ||     |           #
# |   regs--tcu--+  |    |   regs--tcu--+  |    |   regs--tcu--+  |           #
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

base_offset                     = 512 * 1024 * 1024
mod_offset                      = base_offset
mod_size                        = 64 * 1024 * 1024
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

    parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                      metavar="T",
                      help="Stop after T ticks")

    Options.addFSOptions(parser)

    (options, args) = parser.parse_args()

    options.dot_config = ''
    options.mem_watches = {}

    if args:
        print("Error: script doesn't take any positional arguments")
        sys.exit(1)

    return options

def getCacheStr(cache):
    return '%d KiB (%d-way assoc, %d cycles)' % (
        cache.size.value / 1024, cache.assoc, cache.tag_latency
    )

def printConfig(pe, tcupos):
    print('      TCU  =eps:%d, bufsz:%d B, blocksz:%d B, count:%d, tlb:%d' % \
        (pe.tcu.num_endpoints, pe.tcu.buf_size.value, pe.tcu.block_size.value,
         pe.tcu.buf_count, pe.tcu.tlb_entries))

    try:
        print('      L1i$ =%s' % (getCacheStr(pe.l1icache)))
        print('      L1d$ =%s' % (getCacheStr(pe.l1dcache)))
        try:
            print('      L2$  =%s' % (getCacheStr(pe.l2cache)))
        except:
            pass
        try:
            print('      IO$  =%s' % (getCacheStr(pe.iocache)))
        except:
            pass

        str = '      Comp =Core -> '
        if tcupos == 0:
            str += 'TCU -> L1$'
        elif tcupos == 1:
            str += 'L1$ -> TCU'
        else:
            str += 'L1$'
        if hasattr(pe, 'l2cache'):
            str += ' -> L2$'
        if tcupos == 2:
            str += ' -> IO$ -> TCU'
        print(str)
    except:
        try:
            print('      imem =%d KiB' % (int(pe.spm.range.end) / 1024))
            print('      Comp =Core -> TCU -> SPM')
        except:
            pass

def interpose(pe, options, name, port):
    if int(pe.pe_id) in options.mem_watches:
        watch = options.mem_watches[int(pe.pe_id)]
        mon = CommMonitor()
        mon.trace = MemWatchProbe(probe_name="PktResponse", ranges=watch)
        mon.cpu_side_ports = port
        setattr(pe, name, mon)
        return mon.master
    return port

def connectTcuToMem(pe, options, l1size, tcupos):
    dport = interpose(pe, options, 'tcudmon', pe.tcu.dcache_master_port)
    if l1size is None or tcupos == 0:
        iport = interpose(pe, options, 'tcuimon', pe.tcu.icache_master_port)

    if not l1size is None:
        if tcupos == 0:
            pe.l1icache.cpu_side = iport
            pe.l1dcache.cpu_side = dport
        else:
            pe.iocache.cpu_side = dport
    else:
        pe.xbar.cpu_side_ports = iport
        pe.xbar.cpu_side_ports = dport

def connectCuToMem(pe, options, dport, iport=None, l1size=None, tcupos=0):
    dport = interpose(pe, options, 'cu_dmon', dport)
    if not iport is None:
        iport = interpose(pe, options, 'cu_imon', iport)

    if not l1size is None and tcupos > 0:
        if not iport is None:
            pe.l1icache.cpu_side = iport
        pe.l1dcache.cpu_side = dport
    else:
        if not iport is None:
            pe.tcu.icache_slave_port = iport
        pe.tcu.dcache_slave_port = dport

def createPE(noc, options, no, systemType, l1size, l2size, spmsize, tcupos, memPE, epCount):
    CPUClass = ObjectList.cpu_list.get(options.cpu_type)

    # each PE is represented by it's own subsystem
    pe = systemType(mem_mode=CPUClass.memory_mode())
    pe.voltage_domain = VoltageDomain(voltage=options.sys_voltage)
    pe.clk_domain = SrcClockDomain(clock=options.cpu_clock,
                                   voltage_domain=pe.voltage_domain)
    pe.pe_id = no

    if not l2size is None:
        pe.xbar = SystemXBar()
    else:
        pe.xbar = L2XBar()
    pe.xbar.point_of_coherency = True

    pe.tcu = Tcu(max_noc_packet_size='2kB', buf_size='2kB')
    pe.tcu.pe_id = no

    pe.tcu.num_endpoints = epCount
    if tcupos > 0:
        pe.tcu.tlb_entries = 32
    else:
        pe.tcu.tlb_entries = 128

    # connection to noc
    pe.tcu.noc_master_port = noc.cpu_side_ports
    pe.tcu.noc_slave_port  = noc.mem_side_ports

    pe.tcu.slave_region = [AddrRange(0, pe.tcu.mmio_region.start - 1)]

    # for some reason, we need to initialize that here explicitly
    pe.tcu.caches = []

    # create caches
    if not l1size is None:
        pe.l1icache = L1_ICache(size=l1size)
        pe.l1icache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
        pe.l1icache.tag_latency = 4
        pe.l1icache.data_latency = 4
        pe.l1icache.response_latency = 4
        pe.tcu.caches.append(pe.l1icache)

        pe.l1dcache = L1_DCache(size=l1size)
        pe.l1dcache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
        pe.l1dcache.tag_latency = 4
        pe.l1dcache.data_latency = 4
        pe.l1dcache.response_latency = 4
        pe.tcu.caches.append(pe.l1dcache)

        if not l2size is None:
            pe.l2cache = L2Cache(size=l2size)
            pe.l2cache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
            pe.l2cache.tag_latency = 12
            pe.l2cache.data_latency = 12
            pe.l2cache.response_latency = 12
            pe.tcu.caches.append(pe.l2cache)

            pe.l2cache.prefetcher = StridePrefetcher(degree = 16)

            # use a crossbar to connect l1icache and l1dcache to l2cache
            pe.tol2bus = L2XBar()
            pe.l2cache.cpu_side = pe.tol2bus.default
            pe.l2cache.mem_side = pe.xbar.cpu_side_ports

            pe.l1icache.mem_side = pe.tol2bus.cpu_side_ports
            pe.l1dcache.mem_side = pe.tol2bus.cpu_side_ports
        else:
            pe.l1dcache.prefetcher = StridePrefetcher(degree = 16)

            pe.l1icache.mem_side = pe.xbar.cpu_side_ports
            pe.l1dcache.mem_side = pe.xbar.cpu_side_ports

        if tcupos > 0:
            pe.iocache = L1_DCache(size='8kB')
            pe.iocache.tag_latency = 4
            pe.iocache.data_latency = 4
            pe.iocache.response_latency = 4
            pe.tcu.caches.append(pe.iocache)
            if not l2size is None and tcupos == 1:
                pe.iocache.mem_side = pe.tol2bus.cpu_side_ports
            else:
                pe.iocache.mem_side = pe.xbar.cpu_side_ports

        # the TCU handles LLC misses
        pe.tcu.cache_mem_slave_port = pe.xbar.default

        # don't check whether the kernel is in memory because a PE does not have memory in this
        # case, but just a cache that is connected to a different PE
        pe.workload.addr_check = False
    elif not spmsize is None:
        pe.spm = Scratchpad(in_addr_map="true")
        pe.spm.cpu_port = pe.xbar.default
        pe.spm.range = spmsize

    if options.isa == 'riscv':
        pe.tcu.pe_mem_offset = 0x10000000
        if l1size is None and not spmsize is None:
            pe.spm.offset = pe.tcu.pe_mem_offset

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
        pe.tcu.block_size = pe.tcu.max_noc_packet_size
        pe.tcu.buf_size = pe.tcu.max_noc_packet_size

        # disable the TLB
        pe.tcu.tlb_entries = 0
        pe.tcu.cpu_to_cache_latency = 0

    connectTcuToMem(pe, options, l1size, tcupos)

    pe.system_port = pe.xbar.cpu_side_ports

    return pe

def createCorePE(noc, options, no, cmdline, memPE, epCount,
                 l1size=None, l2size=None, tcupos=0, spmsize='8MB'):
    CPUClass = ObjectList.cpu_list.get(options.cpu_type)

    if options.isa == 'arm':
        sysType = M3ArmSystem
        con = ArmConnector
    elif options.isa == 'riscv':
        sysType = M3System
        con = RiscvConnector
    else:
        sysType = M3System
        con = X86Connector

    pe = createPE(
        noc=noc, options=options, no=no, systemType=sysType,
        l1size=l1size, l2size=l2size, spmsize=spmsize, tcupos=tcupos,
        memPE=memPE, epCount=epCount
    )
    pe.tcu.connector = con()
    pe.readfile = "/dev/stdin"

    # connection to the NoC for initialization
    pe.noc_master_port = noc.cpu_side_ports

    pe.cpu = CPUClass()
    pe.cpu.cpu_id = 0

    connectCuToMem(pe, options,
                   pe.cpu.dcache_port,
                   pe.cpu.icache_port,
                   l1size, tcupos)

    # cache misses to MMIO region go to TCU
    if not l1size is None and tcupos > 0:
        pe.tcu.dcache_slave_port = pe.xbar.mem_side_ports
        pe.tcu.slave_region = [pe.tcu.mmio_region]

    if "kernel" in cmdline:
        pe.mod_offset = mod_offset
        pe.mod_size = mod_size
        pe.pe_size = pe_size

    # workload and command line
    if options.isa == 'riscv':
        pe.workload = RiscvBareMetal(bootloader = cmdline.split(' ')[0])
    elif options.isa == 'arm':
        pe.workload = ArmFsWorkload(object_file = cmdline.split(' ')[0])
        pe.highest_el_is_64 = False
    else:
        pe.workload = X86FsWorkload(object_file = cmdline.split(' ')[0])
    pe.cmdline = cmdline

    print("PE%02d: %s" % (no, cmdline))
    print('      Core =%s %s @ %s' % (type(pe.cpu), options.isa, options.cpu_clock))
    printConfig(pe, tcupos)

    # if specified, let this PE wait for GDB
    if options.pausepe == no:
        print('      waiting for GDB')
        pe.cpu.wait_for_remote_gdb = True

    print()

    # connect the IO space via bridge to the root NoC
    pe.bridge = Bridge(delay='50ns')
    pe.bridge.mem_side_port = noc.cpu_side_ports
    pe.bridge.cpu_side_port = pe.xbar.mem_side_ports
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
    #     pe.xbar.mem_side_ports = pe.iobus.cpu_side_ports
    #     pe.pc.attachIO(pe.iobus)

    pe.cpu.createThreads()
    pe.cpu.createInterruptController()

    if options.isa == 'x86_64':
        pe.cpu.interrupts[0].pio = pe.xbar.mem_side_ports
        pe.cpu.interrupts[0].int_responder = pe.tcu.connector.irq_master_port
        pe.cpu.interrupts[0].int_requestor = pe.xbar.cpu_side_ports

    pe.cpu.itb_walker_cache = PageTableWalkerCache()
    pe.cpu.dtb_walker_cache = PageTableWalkerCache()
    pe.cpu.mmu.connectWalkerPorts(pe.cpu.itb_walker_cache.cpu_side, pe.cpu.dtb_walker_cache.cpu_side)

    if not l2size is None:
        pe.cpu.itb_walker_cache.mem_side = pe.tol2bus.cpu_side_ports
        pe.cpu.dtb_walker_cache.mem_side = pe.tol2bus.cpu_side_ports
    else:
        pe.cpu.itb_walker_cache.mem_side = pe.xbar.cpu_side_ports
        pe.cpu.dtb_walker_cache.mem_side = pe.xbar.cpu_side_ports

    return pe

def createSerialPE(noc, options, no, memPE, epCount):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=None, l2size=None, spmsize=None, memPE=memPE,
        tcupos=0, epCount=epCount
    )
    pe.tcu.connector = BaseConnector()

    # the serial device reads from the host's stdin and sends the read bytes
    # via TCU to some defined receive EP
    pe.serial = TcuSerialInput()
    pe.serial.id = no
    pe.serial.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                         voltage_domain=pe.voltage_domain)

    connectCuToMem(pe, options, pe.serial.tcu_master_port)
    pe.serial.tcu_slave_port = pe.xbar.mem_side_ports

    print('PE%02d: SerialInput' % (no))
    printConfig(pe, 0)
    print('      Comp =TCU -> SerialInput')
    print()

    return pe

def createDevicePE(noc, options, no, memPE, epCount):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=None, l2size=None, spmsize=None, memPE=memPE,
        tcupos=0, epCount=epCount
    )
    pe.tcu.connector = BaseConnector()

    # the proxy sits between the device and the TCU to translate interrupts
    # and DMA to the TCU's mechanisms
    pe.proxy = TcuPciProxy()
    pe.proxy.id = no
    pe.proxy.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                         voltage_domain=pe.voltage_domain)

    connectCuToMem(pe, options, pe.proxy.tcu_master_port)
    pe.proxy.tcu_slave_port = pe.xbar.mem_side_ports

    # for the PCI config space
    pe.pci_host = TcuPciHost()
    pe.pci_host.pci_proxy = pe.proxy
    pe.pci_host.conf_base = 0xf000000
    pe.pci_host.conf_size = 0x1000000

    # use bridges to simulate the latency to the I/O devices
    pe.iobus = IOXBar()
    # I/O bridge for requests from the host to the device
    pe.iobridge = Bridge(delay='50ns')
    pe.iobridge.mem_side_port = pe.iobus.cpu_side_ports
    pe.proxy.pio_port = pe.iobridge.cpu_side_port
    pe.pci_host.pio = pe.iobus.mem_side_ports

    # DMA bridge for requests from the device to the host
    pe.dmabridge = Bridge(delay='50ns')
    pe.dmabridge.mem_side_port = pe.proxy.dma_port

    return pe

def createStoragePE(noc, options, no, memPE, epCount, img0=None, img1=None):
    pe = createDevicePE(noc, options, no, memPE, epCount)

    # create disks
    disks = []
    for img in [img0, img1]:
        if img is not None:
            disk = IdeDisk(driveID='device0')
            disk.image = RawDiskImage(image_file = img)
            disks.append(disk)

    pe.idectrl = IdeController(disks=disks,
        pci_func=0, pci_dev=0, pci_bus=0)
    pe.idectrl.BAR0 = PciLegacyIoBar(addr=0x1f0, size='8B')
    pe.idectrl.BAR1 = PciLegacyIoBar(addr=0x3f4, size='3B')
    pe.idectrl.BAR2 = PciLegacyIoBar(addr=0x170, size='8B')
    pe.idectrl.BAR3 = PciLegacyIoBar(addr=0x374, size='3B')
    pe.idectrl.Command = 1
    pe.idectrl.io_shift = 0
    pe.idectrl.InterruptPin = 1
    pe.idectrl.InterruptLine = 14
    pe.idectrl.ctrl_offset = 0

    pe.idectrl.pio = pe.iobus.default
    pe.idectrl.dma = pe.dmabridge.cpu_side_port

    print('pe%02d: %s' % (no, img0))
    printConfig(pe, 0)
    print('      Comp =TCU -> Proxy -> IDE')
    print()

    return pe

def createEtherPE(noc, options, no, memPE, epCount):
    pe = createDevicePE(noc, options, no, memPE, epCount)

    pe.nic = IGbE_e1000()
    pe.nic.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                       voltage_domain=pe.voltage_domain)

    pe.nic.host = pe.pci_host
    pe.nic.pci_bus = 0
    pe.nic.pci_dev = 0
    pe.nic.pci_func = 0

    pe.nic.pio = pe.iobus.default
    pe.nic.dma = pe.dmabridge.cpu_side_port

    print('PE%02d: IGbE_e1000' % (no))
    printConfig(pe, 0)
    print('      Comp =TCU -> Proxy -> NIC')
    print()

    return pe

def linkEtherPEs(ether0, ether1):
    link = EtherLink()
    link.int0 = ether0.nic.interface
    link.int1 = ether1.nic.interface

    ether0.etherlink = link
    ether1.etherlink = link

def createAccelPE(noc, options, no, accel, memPE, epCount,
                  l1size=None, l2size=None, spmsize='64kB'):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memPE=memPE,
        tcupos=0, epCount=epCount
    )
    pe.tcu.connector = TcuAccelConnector()

    if accel == 'indir':
        pe.accel = TcuAccelInDir()
    elif accel == 'copy' or accel == 'rot13':
        algos = {
            'copy'   : 0,
            'rot13'  : 1,
        }
        pe.accel = TcuAccelStream()
        pe.accel.logic = AccelLogic()
        pe.accel.logic.algorithm = algos.get(accel)
        pe.accel.logic.port = pe.xbar.cpu_side_ports
        pe.accel.buf_size = "4kB"
    else:
        print('Accelerator "%s" does not exist' % (accel))
        sys.exit(1)
    pe.tcu.connector.accelerator = pe.accel
    pe.accel.id = no;
    pe.accel.offset = pe.tcu.pe_mem_offset

    connectCuToMem(pe, options, pe.accel.port)

    print('PE%02d: %s accelerator @ %s' % (no, accel, options.cpu_clock))
    printConfig(pe, 0)
    print()

    return pe

def createAbortTestPE(noc, options, no, memPE, epCount,
                      l1size=None, l2size=None, spmsize='8MB'):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memPE=memPE,
        tcupos=0, epCount=epCount
    )
    pe.tcu.connector = BaseConnector()

    pe.cpu = TcuAbortTest()
    pe.cpu.id = no;

    connectCuToMem(pe, options, pe.cpu.port)

    print('PE%02d: aborttest core' % (no))
    printConfig(pe, 0)
    print()

    return pe

def createMemPE(noc, options, no, size, epCount,
                dram=True, image=None, imageNum=0):
    pe = createPE(
        noc=noc, options=options, no=no, systemType=MemSystem,
        l1size=None, l2size=None, spmsize=None, memPE=0,
        tcupos=0, epCount=epCount
    )
    pe.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                   voltage_domain=pe.voltage_domain)
    pe.tcu.connector = BaseConnector()

    # use many buffers to prevent that this is a bottleneck (this is just a
    # simulation artefact anyway)
    pe.tcu.buf_count = 8

    size_bytes = MemorySize(size).value
    if dram:
        pe.mem_ctrl = MemCtrl()
        pe.mem_ctrl.dram = DDR3_1600_8x8()
        pe.mem_ctrl.dram.device_size = size
        pe.mem_ctrl.dram.range = size_bytes
        pe.mem_ctrl.port = pe.xbar.mem_side_ports
    else:
        pe.mem_ctrl = Scratchpad(in_addr_map="true")
        pe.mem_ctrl.cpu_port = pe.xbar.mem_side_ports
        pe.mem_ctrl.range = size_bytes

    if not image is None:
        if os.stat(image).st_size * imageNum > base_offset:
            print('File "%s" is too large for memory layout (%u x %u vs %u)' \
              % (image, imageNum, os.stat(image).st_size, base_offset))
            sys.exit(1)
        pe.mem_file = image
        pe.mem_file_num = imageNum

    print('PE%02d: %s x %d' % (no, image, imageNum))
    printConfig(pe, 0)
    print('      imem =%d KiB' % (int(size_bytes) / 1024))
    name = 'SPM' if type(pe.mem_ctrl).__name__ == 'Scratchpad' else 'DRAM'
    print('      Comp =TCU -> %s' % (name))
    print()

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
    root.noc = IOXBar()

    # create a dummy platform and system for the UART
    root.platform = IOPlatform()
    root.platform.system = System()
    root.platform.system.system_port = root.noc.cpu_side_ports
    root.platform.intrctrl = IntrControl()

    # UART and terminal
    root.platform.com_1 = Uart8250()
    root.platform.com_1.pio_addr = IO_address_space_base + 0x3f8
    root.platform.com_1.device = Terminal()
    root.platform.com_1.pio = root.noc.mem_side_ports

    return root

def runSimulation(root, options, pes):
    # determine types of PEs and their internal memory size
    pemems = []
    for pe in pes:
        size = 0
        if hasattr(pe, 'mem_ctrl'):
            size = int(pe.mem_ctrl.dram.device_size)
            assert size % 4096 == 0, "Memory size not page aligned"
            size |= 2   # mem
        else:
            if hasattr(pe, 'spm'):
                size = int(pe.spm.range.end)
                assert size % 4096 == 0, "Memory size not page aligned"
            else:
                size |= 1 # emem

            if hasattr(pe, 'accel'):
                if type(pe.accel).__name__ == 'TcuAccelInDir':
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
            elif hasattr(pe, 'serial'):
                size = 9 << 3
        pemems.append(size)

    # give that to the PEs
    for pe in pes:
        setattr(root, 'pe%02d' % pe.pe_id, pe)
        try:
            pe.mods = options.mods
            pe.pes = pemems
        except:
            pass

    sys.stdout.flush()

    # Instantiate configuration
    m5.instantiate()

    # Simulate until program terminates
    exit_event = m5.simulate(options.maxtick)

    print('Exiting @ tick', m5.curTick(), 'because', exit_event.getCause())
