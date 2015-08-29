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
    cpu_slave_port  = SlavePort("DTU slave port connecting to the CPU")
    spm_master_port = MasterPort("DTU master port connecting to the Scratchpad Memory")
    noc_master_port = MasterPort("DTU master port")
    noc_slave_port  = SlavePort("DTU slave port")

    regfile_base_addr = Param.Addr(0x1000000, "Register file address")
    noc_addr_width = Param.Unsigned(32, "NoC address width")
    noc_core_addr_bits = Param.Unsigned(5, "Number of bits used to address a core")
    noc_ep_addr_bits = Param.Unsigned(5, "Number of bits used address an endpoint within a core")
    core_id = Param.Unsigned("ID of the core this DTU belongs to")

class Dtu(BaseDtu):
    type = 'Dtu'
    cxx_header = "mem/dtu/dtu.hh"
    system = Param.System(Parent.any, "System we belong to")
    use_ptable = Param.Bool('false', "If true all address are translated into physical addresses")
    num_endpoints = Param.Unsigned(8, "Number of enpoints per DTU")

    num_cmd_epid_bits = Param.Unsigned(8, "Number of bits used to identify the endpoint in a command")
    num_cmd_offset_bits = Param.Unsigned(16, "Number of bits used for address offset on memory access")

    max_noc_packet_size = Param.MemorySize("512B", "Maximum size of a NoC packet")

    register_access_latency = Param.Cycles(1, "Latency for CPU register accesses")
    command_to_spm_request_latency = Param.Cycles(5, "Number of cycles passed from writing a command to the register to issuing a read request on the scratchpad port")
    command_to_noc_request_latency = Param.Cycles(5, "Number of cycles passed from writing a command to the register to issuing a read request on the NoC port")
    spm_response_to_noc_request_latency = Param.Cycles(1, "Number of cycles passed from receiving data on the scratchpad port to sending it on the NoC port")
    noc_message_to_spm_request_latency = Param.Cycles(3, "Number of cycles passsed from receiving a message on the noc port until forwarding it to the spm port");
    noc_response_to_spm_request_latency = Param.Cycles(3, "Number of cycles passsed from receiving a response on the noc port until forwarding it to the spm port");
    noc_request_to_spm_request_latency = Param.Cycles(2, "Number of cycles passsed from receiving a request on the noc port until forwarding it to the spm port");
    spm_response_to_noc_response_latency = Param.Cycles(1, "Number of cycles passed from receiving a response from the schratchpad to forwarding it to the NoC");
