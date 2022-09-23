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

from m5.objects.ClockedObject import ClockedObject
from m5.objects.Connector import BaseConnector
from m5.params import *
from m5.proxy import *

class BaseTcu(ClockedObject):
    type = 'BaseTcu'
    abstract = True
    cxx_header = "mem/tcu/base.hh"

    system = Param.System(Parent.any, "System we belong to")

    noc_master_port = MasterPort("TCU master port")
    noc_slave_port  = SlavePort("TCU slave port")

    icache_slave_port = SlavePort("Port that forwards requests from the CPU to the icache")
    dcache_slave_port = SlavePort("Port that forwards requests from the CPU to the dcache")
    slave_region = VectorParam.AddrRange([], "The address region for requests to the TCU")

    icache_master_port = MasterPort("Port that connects the icache")
    dcache_master_port = MasterPort("Port that connects the dcache")

    llc_slave_port = SlavePort("Port that performs memory requests on behalf of the cache")

    tile_mem_offset = Param.Unsigned(0, "The offset that all accesses have to go above")

    mmio_region = Param.AddrRange(AddrRange(0xF0000000, 0xF0003FFF), "MMIO region of the TCU")

    tile_id = Param.Unsigned("ID of the tile this TCU belongs to")

    watch_range_start = Param.Addr(0x0, "The start address of the address range to watch")
    watch_range_end = Param.Addr(0x0, "The end address of the address range to watch (exclusive)")

class Tcu(BaseTcu):
    type = 'Tcu'
    cxx_header = "mem/tcu/tcu.hh"
    num_endpoints = Param.Unsigned(8, "Number of endpoints per TCU")

    connector = Param.BaseConnector("The connector to the CPU")

    max_noc_packet_size = Param.MemorySize("1kB", "Maximum size of a NoC packet (needs to be the same for all TCUs)")

    tlb_entries = Param.Unsigned(32, "The number of TLB entries")

    block_size = Param.MemorySize("64B", "The block size with which to access the local memory")

    buf_count = Param.Unsigned(4, "The number of temporary buffers for transfers")
    buf_size = Param.MemorySize("1kB", "The size of a temporary buffer")
    req_count = Param.Unsigned(4, "The number of parallel requests to memory")

    mmio_latency = Param.Cycles(1, "Latency for MMIO-based register accesses")
    cpu_to_cache_latency = Param.Cycles(1, "Latency for cache access for the CPU")
    tlb_latency = Param.Cycles(3, "Latency to access the TLB")

    cmd_read_latency = Param.Cycles(20, "Latency for the READ command (start until NoC request)")
    cmd_write_latency = Param.Cycles(14, "Latency for the WRITE command (start until NoC request)")
    cmd_send_latency = Param.Cycles(21, "Latency for the SEND command (start until NoC request)")
    cmd_reply_latency = Param.Cycles(25, "Latency for the REPLY command (start until NoC request)")
    cmd_recv_latency = Param.Cycles(20, "Latency for receiving a message")
