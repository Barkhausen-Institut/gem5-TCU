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

from MemObject import MemObject
from m5.params import *
from m5.proxy import *


class BaseDtu(MemObject):
    type = 'BaseDtu'
    abstract = True
    cxx_header = "mem/dtu/base.hh"
    noc_master_port = MasterPort("DTU master port")
    noc_slave_port  = SlavePort("DTU slave port")

    icache_slave_port = SlavePort("Port that forwards requests from the CPU to the icache")
    dcache_slave_port = SlavePort("Port that forwards requests from the CPU to the icache")

    icache_master_port = MasterPort("Port that connects the icache")
    dcache_master_port = MasterPort("Port that connects the dcache")

    cache_mem_slave_port = SlavePort("Port that performs memory requests on behalf of the cache")

    regfile_base_addr = Param.Addr(0xF0000000, "Register file address")
    core_id = Param.Unsigned("ID of the core this DTU belongs to")

    watch_range_start = Param.Addr(0x0, "The start address of the address range to watch")
    watch_range_end = Param.Addr(0x0, "The end address of the address range to watch (exclusive)")

    tlb_entries = Param.Unsigned(512, "The number of TLB entries");
    page_bits = Param.Unsigned(12, "The page size (1 << page_bits)");

class Dtu(BaseDtu):
    type = 'Dtu'
    cxx_header = "mem/dtu/dtu.hh"
    system = Param.System(Parent.any, "System we belong to")
    num_endpoints = Param.Unsigned(8, "Number of enpoints per DTU")

    num_cmd_epid_bits = Param.Unsigned(8, "Number of bits used to identify the endpoint in a command")

    max_noc_packet_size = Param.MemorySize("1kB", "Maximum size of a NoC packet (needs to be the same for all DTUs)")

    memory_ep = Param.Unsigned(7, "The memory endpoint")
    memory_pe = Param.Unsigned(0, "The memory PE to use")
    memory_offset = Param.Addr(0, "The offset in the memory PE")
    memory_size = Param.MemorySize("0kB", "The size of the memory range in the memory PE")

    block_size = Param.MemorySize("64B", "The block size with which to access the local memory")

    buf_count = Param.Unsigned(4, "The number of temporary buffers for transfers")
    buf_size = Param.MemorySize("1kB", "The size of a temporary buffer")

    register_access_latency = Param.Cycles(1, "Latency for CPU register accesses")
    
    command_to_noc_request_latency = Param.Cycles(1, "Number of cycles passed from writing a command to the register to starting the command")
    start_msg_transfer_delay = Param.Cycles(2, "Number of cycles passed to build the header and start the message transfer")

    transfer_to_mem_request_latency = Param.Cycles(1, "Number of cycles passed for requesting something from local memory, when transferring")
    transfer_to_noc_latency = Param.Cycles(3, "Number of cycles passed from collecting the data in the buffer until sending it to the NoC");
    noc_to_transfer_latency = Param.Cycles(3, "Number of cycles passed from receiving data from the NoC until starting to transfer it to the local memory");
