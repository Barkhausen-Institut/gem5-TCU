# Copyright (c) 2015 Christian Menard
# Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
# Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
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

# Each tile is represented as an instance of System. Whereas each tile has a
# CPU, a Scratchpad, and a TCU. Because it seems that the gem5 crossbar is not
# able to handle requests from the icache/dcache ports of the CPU if using the
# O3 model, we're connecting the icache/dcache ports to the TCU. The TCU
# forwards the request either to its own register file or the scratchpad,
# depending on the address range. The tiles are connected via a global
# crossbar.

###############################################################################
# root                                                                        #
#                                                                             #
# |-----------------|    |-----------------|    |-----------------|           #
# | T0              |    | T1              |    | T2              |           #
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

base_offset                     = 768 * 1024 * 1024
mod_offset                      = base_offset
mod_size                        = 64 * 1024 * 1024
tile_offset                     = mod_offset + mod_size
tile_size                       = 16 * 1024 * 1024

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

    parser.add_option("--pausetile", default=-1, type="int",
                      help="the tile to pause until GDB connects")

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

def printConfig(tile, tcupos):
    print('     TCU  =eps:%d, bufsz:%d B, blocksz:%d B, count:%d, tlb:%d' % \
        (tile.tcu.num_endpoints, tile.tcu.buf_size.value, tile.tcu.block_size.value,
         tile.tcu.buf_count, tile.tcu.tlb_entries))

    try:
        print('     L1i$ =%s' % (getCacheStr(tile.l1icache)))
        print('     L1d$ =%s' % (getCacheStr(tile.l1dcache)))
        try:
            print('     L2$  =%s' % (getCacheStr(tile.l2cache)))
        except:
            pass
        try:
            print('     IO$  =%s' % (getCacheStr(tile.iocache)))
        except:
            pass

        str = '     Comp =Core -> '
        if tcupos == 0:
            str += 'TCU -> L1$'
        elif tcupos == 1:
            str += 'L1$ -> TCU'
        else:
            str += 'L1$'
        if hasattr(tile, 'l2cache'):
            str += ' -> L2$'
        if tcupos == 2:
            str += ' -> IO$ -> TCU'
        print(str)
    except:
        try:
            print('     imem =%d KiB' % (int(tile.spm.range.end) / 1024))
            print('     Comp =Core -> TCU -> SPM')
        except:
            pass

def interpose(tile, options, name, port):
    if int(tile.tile_id) in options.mem_watches:
        watch = options.mem_watches[int(tile.tile_id)]
        mon = CommMonitor()
        mon.trace = MemWatchProbe(probe_name="PktResponse", ranges=watch)
        mon.cpu_side_ports = port
        setattr(tile, name, mon)
        return mon.master
    return port

def connectTcuToMem(tile, options, l1size, tcupos):
    dport = interpose(tile, options, 'tcudmon', tile.tcu.dcache_master_port)
    if l1size is None or tcupos == 0:
        iport = interpose(tile, options, 'tcuimon', tile.tcu.icache_master_port)

    if not l1size is None:
        if tcupos == 0:
            tile.l1icache.cpu_side = iport
            tile.l1dcache.cpu_side = dport
        else:
            tile.iocache.cpu_side = dport
    else:
        tile.xbar.cpu_side_ports = iport
        tile.xbar.cpu_side_ports = dport

def connectCuToMem(tile, options, dport, iport=None, l1size=None, tcupos=0):
    dport = interpose(tile, options, 'cu_dmon', dport)
    if not iport is None:
        iport = interpose(tile, options, 'cu_imon', iport)

    if not l1size is None and tcupos > 0:
        if not iport is None:
            tile.l1icache.cpu_side = iport
        tile.l1dcache.cpu_side = dport
    else:
        if not iport is None:
            tile.tcu.icache_slave_port = iport
        tile.tcu.dcache_slave_port = dport

def createTile(noc, options, no, systemType, l1size, l2size, spmsize,
               tcupos, memTile, epCount):
    CPUClass = ObjectList.cpu_list.get(options.cpu_type)

    # each tile is represented by it's own subsystem
    tile = systemType(mem_mode=CPUClass.memory_mode())
    tile.voltage_domain = VoltageDomain(voltage=options.sys_voltage)
    tile.clk_domain = SrcClockDomain(clock=options.cpu_clock,
                                   voltage_domain=tile.voltage_domain)
    tile.tile_id = no

    if not l2size is None:
        tile.xbar = SystemXBar()
    else:
        tile.xbar = L2XBar()
    tile.xbar.point_of_coherency = True

    tile.tcu = Tcu(max_noc_packet_size='2kB', buf_size='2kB')
    tile.tcu.tile_id = no

    tile.tcu.num_endpoints = epCount
    if tcupos > 0:
        tile.tcu.tlb_entries = 32
    else:
        tile.tcu.tlb_entries = 128

    # connection to noc
    tile.tcu.noc_master_port = noc.cpu_side_ports
    tile.tcu.noc_slave_port  = noc.mem_side_ports

    tile.tcu.slave_region = [AddrRange(0, tile.tcu.mmio_region.start - 1)]

    # for some reason, we need to initialize that here explicitly
    tile.tcu.caches = []

    # create caches
    if not l1size is None:
        tile.l1icache = L1_ICache(size=l1size)
        tile.l1icache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
        tile.l1icache.tag_latency = 4
        tile.l1icache.data_latency = 4
        tile.l1icache.response_latency = 4
        tile.tcu.caches.append(tile.l1icache)

        tile.l1dcache = L1_DCache(size=l1size)
        tile.l1dcache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
        tile.l1dcache.tag_latency = 4
        tile.l1dcache.data_latency = 4
        tile.l1dcache.response_latency = 4
        tile.tcu.caches.append(tile.l1dcache)

        if not l2size is None:
            tile.l2cache = L2Cache(size=l2size)
            tile.l2cache.addr_ranges = [AddrRange(0, 0x1000000000000000 - 1)]
            tile.l2cache.tag_latency = 12
            tile.l2cache.data_latency = 12
            tile.l2cache.response_latency = 12
            tile.tcu.caches.append(tile.l2cache)

            tile.l2cache.prefetcher = StridePrefetcher(degree = 16)

            # use a crossbar to connect l1icache and l1dcache to l2cache
            tile.tol2bus = L2XBar()
            tile.l2cache.cpu_side = tile.tol2bus.default
            tile.l2cache.mem_side = tile.xbar.cpu_side_ports

            tile.l1icache.mem_side = tile.tol2bus.cpu_side_ports
            tile.l1dcache.mem_side = tile.tol2bus.cpu_side_ports
        else:
            tile.l1dcache.prefetcher = StridePrefetcher(degree = 16)

            tile.l1icache.mem_side = tile.xbar.cpu_side_ports
            tile.l1dcache.mem_side = tile.xbar.cpu_side_ports

        if tcupos > 0:
            tile.iocache = L1_DCache(size='8kB')
            tile.iocache.tag_latency = 4
            tile.iocache.data_latency = 4
            tile.iocache.response_latency = 4
            tile.tcu.caches.append(tile.iocache)
            if not l2size is None and tcupos == 1:
                tile.iocache.mem_side = tile.tol2bus.cpu_side_ports
            else:
                tile.iocache.mem_side = tile.xbar.cpu_side_ports

        # the TCU handles LLC misses
        tile.tcu.cache_mem_slave_port = tile.xbar.default

        # don't check whether the kernel is in memory because a tile does not have memory in this
        # case, but just a cache that is connected to a different tile
        tile.workload.addr_check = False
    elif not spmsize is None:
        tile.spm = Scratchpad(in_addr_map="true")
        tile.spm.cpu_port = tile.xbar.default
        tile.spm.range = spmsize

    if options.isa == 'riscv':
        tile.tcu.tile_mem_offset = 0x10000000
        if l1size is None and not spmsize is None:
            tile.spm.offset = tile.tcu.tile_mem_offset

    if systemType != MemSystem:
        tile.memory_tile = memTile
        tile.memory_offset = tile_offset + (tile_size * no)
        tile.memory_size = tile_size

    if systemType == MemSystem or l1size is None:
        # for memory tiles or tiles with SPM, we do not need a buffer. for the
        # sake of an easy implementation we just make the buffer very large and
        # the block size as well, so that we can read a packet from SPM/DRAM
        # into the buffer and send it from there. Since that costs no simulated
        # time, it is the same as having no buffer.
        tile.tcu.block_size = tile.tcu.max_noc_packet_size
        tile.tcu.buf_size = tile.tcu.max_noc_packet_size

        # disable the TLB
        tile.tcu.tlb_entries = 0
        tile.tcu.cpu_to_cache_latency = 0

    connectTcuToMem(tile, options, l1size, tcupos)

    tile.system_port = tile.xbar.cpu_side_ports

    return tile

def createCoreTile(noc, options, no, cmdline, memTile, epCount,
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

    tile = createTile(
        noc=noc, options=options, no=no, systemType=sysType,
        l1size=l1size, l2size=l2size, spmsize=spmsize, tcupos=tcupos,
        memTile=memTile, epCount=epCount
    )
    tile.tcu.connector = con()
    tile.readfile = "/dev/stdin"

    # connection to the NoC for initialization
    tile.noc_master_port = noc.cpu_side_ports

    tile.cpu = CPUClass()
    tile.cpu.cpu_id = 0

    connectCuToMem(tile, options,
                   tile.cpu.dcache_port,
                   tile.cpu.icache_port,
                   l1size, tcupos)

    # cache misses to MMIO region go to TCU
    if not l1size is None and tcupos > 0:
        tile.tcu.dcache_slave_port = tile.xbar.mem_side_ports
        tile.tcu.slave_region = [tile.tcu.mmio_region]

    if "kernel" in cmdline:
        tile.mod_offset = mod_offset
        tile.mod_size = mod_size
        tile.tile_size = tile_size

    # workload and command line
    if options.isa == 'riscv':
        tile.workload = RiscvBareMetal(bootloader = cmdline.split(' ')[0])
    elif options.isa == 'arm':
        tile.workload = ArmFsWorkload(object_file = cmdline.split(' ')[0])
        tile.highest_el_is_64 = False
    else:
        tile.workload = X86FsWorkload(object_file = cmdline.split(' ')[0])
    tile.cmdline = cmdline

    print("T%02d: %s" % (no, cmdline))
    print('     Core =%s %s @ %s' % (type(tile.cpu), options.isa, options.cpu_clock))
    printConfig(tile, tcupos)

    # if specified, let this tile wait for GDB
    if options.pausetile == no:
        print('      waiting for GDB')
        tile.cpu.wait_for_remote_gdb = True

    print()

    # connect the IO space via bridge to the root NoC
    tile.bridge = Bridge(delay='50ns')
    tile.bridge.mem_side_port = noc.cpu_side_ports
    tile.bridge.cpu_side_port = tile.xbar.mem_side_ports
    tile.bridge.ranges = \
        [
        AddrRange(IO_address_space_base,
                  interrupts_address_space_base - 1)
        ]

    # if not l1size is None:
    #     # connect legacy devices
    #     tile.pc = Pc()
    #     tile.intrctrl = IntrControl()
    #     tile.iobus = IOXBar()
    #     tile.xbar.mem_side_ports = tile.iobus.cpu_side_ports
    #     tile.pc.attachIO(tile.iobus)

    tile.cpu.createThreads()
    tile.cpu.createInterruptController()

    if options.isa == 'x86_64':
        tile.cpu.interrupts[0].pio = tile.xbar.mem_side_ports
        tile.cpu.interrupts[0].int_responder = tile.tcu.connector.irq_master_port
        tile.cpu.interrupts[0].int_requestor = tile.xbar.cpu_side_ports

    tile.cpu.itb_walker_cache = PageTableWalkerCache()
    tile.cpu.dtb_walker_cache = PageTableWalkerCache()
    tile.cpu.mmu.connectWalkerPorts(tile.cpu.itb_walker_cache.cpu_side, tile.cpu.dtb_walker_cache.cpu_side)

    if options.isa == 'riscv':
        tile.cpu.mmu.pma_checker = PMAChecker(uncacheable = [
            tile.tcu.mmio_region,
        ])

    if not l2size is None:
        tile.cpu.itb_walker_cache.mem_side = tile.tol2bus.cpu_side_ports
        tile.cpu.dtb_walker_cache.mem_side = tile.tol2bus.cpu_side_ports
    else:
        tile.cpu.itb_walker_cache.mem_side = tile.xbar.cpu_side_ports
        tile.cpu.dtb_walker_cache.mem_side = tile.xbar.cpu_side_ports

    return tile

def createKecAccTile(noc, options, no, cmdline, memTile, epCount, spmsize='8MB'):
    tile = createCoreTile(noc, options, no, cmdline, memTile, epCount,
                          spmsize=spmsize)

    # The MMIO address of the accelerator
    addr = 0xF4200000

    # Disable extra PIO latency since it causes quite some overhead that should
    # not be present in reality if the accelerator is very close to the CPU.
    tile.kecacc = KecAcc(pio_addr=addr, pio_latency='0')
    tile.kecacc.pio = tile.xbar.mem_side_ports

    # FIXME: KecAcc and TCU work in parallel to implement double buffering.
    # In real hardware this would likely be implemented using separate SPMs for
    # code/data of the Core and the buffers. For simplicity this is currently
    # simulated with a single SPM here. Unfortunately this seems to cause
    # unintended delays in the XBar when both access the SPM at the same time.
    # To avoid this, connect KecAcc directly to a separate port on the SPM
    # for now. It looks like a separate port was used for the TCU at some point
    # (but not anymore), so we can "abuse" it here as second port on the SPM.
    tile.kecacc.port = tile.spm.tcu_port

    # Make sure accelerator is accessed uncached and without speculation
    if options.isa == 'riscv':
        tile.cpu.mmu.pma_checker.uncacheable.append(AddrRange(addr, size=0x1000))

    return tile

def createSerialTile(noc, options, no, memTile, epCount):
    tile = createTile(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=None, l2size=None, spmsize=None, memTile=memTile,
        tcupos=0, epCount=epCount
    )
    tile.tcu.connector = BaseConnector()

    # the serial device reads from the host's stdin and sends the read bytes
    # via TCU to some defined receive EP
    tile.serial = TcuSerialInput()
    tile.serial.id = no
    tile.serial.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                            voltage_domain=tile.voltage_domain)

    connectCuToMem(tile, options, tile.serial.tcu_master_port)
    tile.serial.tcu_slave_port = tile.xbar.mem_side_ports

    print('T%02d: SerialInput' % (no))
    printConfig(tile, 0)
    print('     Comp =TCU -> SerialInput')
    print()

    return tile

def createDeviceTile(noc, options, no, memTile, epCount):
    tile = createTile(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=None, l2size=None, spmsize=None, memTile=memTile,
        tcupos=0, epCount=epCount
    )
    tile.tcu.connector = BaseConnector()

    # the proxy sits between the device and the TCU to translate interrupts
    # and DMA to the TCU's mechanisms
    tile.proxy = TcuPciProxy()
    tile.proxy.id = no
    tile.proxy.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                           voltage_domain=tile.voltage_domain)

    connectCuToMem(tile, options, tile.proxy.tcu_master_port)
    tile.proxy.tcu_slave_port = tile.xbar.mem_side_ports

    # for the PCI config space
    tile.pci_host = TcuPciHost()
    tile.pci_host.pci_proxy = tile.proxy
    tile.pci_host.conf_base = 0xf000000
    tile.pci_host.conf_size = 0x1000000

    # use bridges to simulate the latency to the I/O devices
    tile.iobus = IOXBar()
    # I/O bridge for requests from the host to the device
    tile.iobridge = Bridge(delay='50ns')
    tile.iobridge.mem_side_port = tile.iobus.cpu_side_ports
    tile.proxy.pio_port = tile.iobridge.cpu_side_port
    tile.pci_host.pio = tile.iobus.mem_side_ports

    # DMA bridge for requests from the device to the host
    tile.dmabridge = Bridge(delay='50ns')
    tile.dmabridge.mem_side_port = tile.proxy.dma_port

    return tile

def createStorageTile(noc, options, no, memTile, epCount, img0=None, img1=None):
    tile = createDeviceTile(noc, options, no, memTile, epCount)

    # create disks
    disks = []
    for img in [img0, img1]:
        if img is not None:
            disk = IdeDisk(driveID='device0')
            disk.image = RawDiskImage(image_file = img)
            disks.append(disk)

    tile.idectrl = IdeController(disks=disks,
        pci_func=0, pci_dev=0, pci_bus=0)
    tile.idectrl.BAR0 = PciLegacyIoBar(addr=0x1f0, size='8B')
    tile.idectrl.BAR1 = PciLegacyIoBar(addr=0x3f4, size='3B')
    tile.idectrl.BAR2 = PciLegacyIoBar(addr=0x170, size='8B')
    tile.idectrl.BAR3 = PciLegacyIoBar(addr=0x374, size='3B')
    tile.idectrl.Command = 1
    tile.idectrl.io_shift = 0
    tile.idectrl.InterruptPin = 1
    tile.idectrl.InterruptLine = 14
    tile.idectrl.ctrl_offset = 0

    tile.idectrl.pio = tile.iobus.default
    tile.idectrl.dma = tile.dmabridge.cpu_side_port

    print('tile%02d: %s' % (no, img0))
    printConfig(tile, 0)
    print('     Comp =TCU -> Proxy -> IDE')
    print()

    return tile

def createEtherTile(noc, options, no, memTile, epCount):
    tile = createDeviceTile(noc, options, no, memTile, epCount)

    tile.nic = IGbE_e1000()
    tile.nic.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                         voltage_domain=tile.voltage_domain)

    tile.nic.host = tile.pci_host
    tile.nic.pci_bus = 0
    tile.nic.pci_dev = 0
    tile.nic.pci_func = 0

    tile.nic.pio = tile.iobus.default
    tile.nic.dma = tile.dmabridge.cpu_side_port

    print('T%02d: IGbE_e1000' % (no))
    printConfig(tile, 0)
    print('     Comp =TCU -> Proxy -> NIC')
    print()

    return tile

def linkEthertiles(ether0, ether1):
    link = EtherLink()
    link.int0 = ether0.nic.interface
    link.int1 = ether1.nic.interface

    ether0.etherlink = link
    ether1.etherlink = link

def createAccelTile(noc, options, no, accel, memTile, epCount,
                    l1size=None, l2size=None, spmsize='64kB'):
    tile = createTile(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memTile=memTile,
        tcupos=0, epCount=epCount
    )
    tile.tcu.connector = TcuAccelConnector()

    if accel == 'indir':
        tile.accel = TcuAccelInDir()
    elif accel == 'copy' or accel == 'rot13':
        algos = {
            'copy'   : 0,
            'rot13'  : 1,
        }
        tile.accel = TcuAccelStream()
        tile.accel.logic = AccelLogic()
        tile.accel.logic.algorithm = algos.get(accel)
        tile.accel.logic.port = tile.xbar.cpu_side_ports
        tile.accel.buf_size = "4kB"
    else:
        print('Accelerator "%s" does not exist' % (accel))
        sys.exit(1)
    tile.tcu.connector.accelerator = tile.accel
    tile.accel.id = no;
    tile.accel.offset = tile.tcu.tile_mem_offset

    connectCuToMem(tile, options, tile.accel.port)

    print('T%02d: %s accelerator @ %s' % (no, accel, options.cpu_clock))
    printConfig(tile, 0)
    print()

    return tile

def createAbortTestTile(noc, options, no, memTile, epCount,
                        l1size=None, l2size=None, spmsize='8MB'):
    tile = createTile(
        noc=noc, options=options, no=no, systemType=SpuSystem,
        l1size=l1size, l2size=l2size, spmsize=spmsize, memTile=memTile,
        tcupos=0, epCount=epCount
    )
    tile.tcu.connector = BaseConnector()

    tile.cpu = TcuAbortTest()
    tile.cpu.id = no;

    connectCuToMem(tile, options, tile.cpu.port)

    print('T%02d: aborttest core' % (no))
    printConfig(tile, 0)
    print()

    return tile

def createMemTile(noc, options, no, size, epCount,
                  dram=True, image=None, imageNum=0):
    tile = createTile(
        noc=noc, options=options, no=no, systemType=MemSystem,
        l1size=None, l2size=None, spmsize=None, memTile=0,
        tcupos=0, epCount=epCount
    )
    tile.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                     voltage_domain=tile.voltage_domain)
    tile.tcu.connector = BaseConnector()

    # use many buffers to prevent that this is a bottleneck (this is just a
    # simulation artefact anyway)
    tile.tcu.buf_count = 8

    size_bytes = MemorySize(size).value
    if dram:
        tile.mem_ctrl = MemCtrl()
        tile.mem_ctrl.dram = DDR3_1600_8x8()
        tile.mem_ctrl.dram.device_size = size
        tile.mem_ctrl.dram.range = size_bytes
        tile.mem_ctrl.port = tile.xbar.mem_side_ports
    else:
        tile.mem_ctrl = Scratchpad(in_addr_map="true")
        tile.mem_ctrl.cpu_port = tile.xbar.mem_side_ports
        tile.mem_ctrl.range = size_bytes

    if not image is None:
        if os.stat(image).st_size * imageNum > base_offset:
            print('File "%s" is too large for memory layout (%u x %u vs %u)' \
              % (image, imageNum, os.stat(image).st_size, base_offset))
            sys.exit(1)
        tile.mem_file = image
        tile.mem_file_num = imageNum

    print('T%02d: %s x %d' % (no, image, imageNum))
    printConfig(tile, 0)
    print('     imem =%d KiB' % (int(size_bytes) / 1024))
    name = 'SPM' if type(tile.mem_ctrl).__name__ == 'Scratchpad' else 'DRAM'
    print('     Comp =TCU -> %s' % (name))
    print()

    return tile

def createRoot(options):
    root = Root(full_system=True)

    # Create a top-level voltage domain
    root.voltage_domain = VoltageDomain(voltage=options.sys_voltage)

    # Create a source clock for the system and set the clock period
    root.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                     voltage_domain=root.voltage_domain)

    # All tiles are connected to a NoC (Network on Chip). In this case it's just
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

def runSimulation(root, options, tiles):
    # determine types of tiles and their internal memory size
    tile_mems = []
    for tile in tiles:
        size = 0
        if hasattr(tile, 'mem_ctrl'):
            size = int(tile.mem_ctrl.dram.device_size)
            assert size % 4096 == 0, "Memory size not page aligned"
            size |= 2   # mem
        else:
            if hasattr(tile, 'spm'):
                size = int(tile.spm.range.end)
                assert size % 4096 == 0, "Memory size not page aligned"
            else:
                size |= 1 # emem

            if hasattr(tile, 'accel'):
                if type(tile.accel).__name__ == 'TcuAccelInDir':
                    size |= 4 << 3 # indir accelerator
                elif int(tile.accel.logic.algorithm) == 0:
                    size |= 5 << 3 # copy accelerator
                elif int(tile.accel.logic.algorithm) == 1:
                    size |= 6 << 3 # rot13 accelerator
            elif options.isa == 'arm':
                size |= 2 << 3 # arm
            elif options.isa == 'riscv':
                size |= 3 << 3 # riscv
            else:
                size |= 1 << 3 # x86

            if hasattr(tile, 'idectrl'):
                size = 7 << 3
            elif hasattr(tile, 'nic'):
                size = 8 << 3
            elif hasattr(tile, 'serial'):
                size = 9 << 3

            if hasattr(tile, 'kecacc'):
                size |= 0x8 << 7 # TileAttr::KECACC
        tile_mems.append(size)

    # give that to the tiles
    for tile in tiles:
        setattr(root, 'T%02d' % tile.tile_id, tile)
        try:
            tile.mods = options.mods
            tile.tiles = tile_mems
        except:
            pass

    sys.stdout.flush()

    # Instantiate configuration
    m5.instantiate()

    # Simulate until program terminates
    exit_event = m5.simulate(options.maxtick)

    print('Exiting @ tick', m5.curTick(), 'because', exit_event.getCause())
