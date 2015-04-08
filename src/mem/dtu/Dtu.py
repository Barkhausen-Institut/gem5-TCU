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

from MemObject import MemObject
from m5.params import *
from m5.proxy import *


class BaseDtu(MemObject):
    type = 'BaseDtu'
    abstract = True
    cxx_header = "mem/dtu/base.hh"
    cpu_base_addr = Param.Addr(0x10000000, "DTU address (used by CPU to access the DTU)")
    dtu_addr_bits = Param.Unsigned(4, "Address bits used to address the DTU")
    dtu_addr = Param.Addr("DTU address (used to access the DTU from memory bus)")
    spm_pkt_size = Param.Unsigned(16, "Packet size in bytes used for communication with the SPM")
    noc_pkt_size = Param.Unsigned(64, "Packte size in bytes used for the NoC")
    system = Param.System(Parent.any, "System this tester is part of")

class Dtu(BaseDtu):
    type = 'Dtu'
    cxx_header = "mem/dtu/dtu.hh"
    cpu        = SlavePort("DTU slave port connectting to the CPU")
    scratchpad = MasterPort("DTU master port connecting to the Scratchpad Memory")
    master     = MasterPort("DTU master port")
    slave      = SlavePort("DTU slave port")
