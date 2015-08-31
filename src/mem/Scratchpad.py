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

from m5.params import *
from AbstractMemory import *

class Scratchpad(AbstractMemory):
    type = 'Scratchpad'
    cxx_header = 'mem/scratchpad.hh'

    range = 0x1000000 # default size 16MiB

    cpu_port = SlavePort("Slave port CPU side")
    dtu_port = SlavePort("Slave port DTU side")

    latency = Param.Cycles(1, "Request to response latency")

    throughput = Param.Unsigned(64, "Number of bytes that can be read per cycle")

    init_file = Param.String("", "File that should be loaded into the scratchpad")

    watch_range_start = Param.Addr(0x0, "The start address of the address range to watch")
    watch_range_end = Param.Addr(0x0, "The end address of the address range to watch (exclusive)")

    # TODO check what these options actually mean, switch them off for now
    in_addr_map = False
    conf_table_reported = False
