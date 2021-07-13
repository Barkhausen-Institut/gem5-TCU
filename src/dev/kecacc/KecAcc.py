# Copyright (c) 2021 Stephan Gerhold
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.params import *
from m5.objects.Device import BasicPioDevice

class KecAcc(BasicPioDevice):
    type = 'KecAcc'
    cxx_header = "dev/kecacc/kecacc.hh"

    port = RequestPort("Port to the memory system")
    buf_size = Param.MemorySize('512KiB', "Maximum size of processable buffers")

    mem_width = Param.Unsigned(16, "Memory width (bytes read/written per cycle)")
    pipeline_cycles = Param.Cycles(2, "Cycles for load pipeline overhead")
    init_cycles = Param.Cycles(1, "Cycles for initializing state")
    load_cycles = Param.Cycles(1, "Cycles for loading state (without memory)")
    save_cycles = Param.Cycles(1, "Cycles for saving state (without memory)")
    permute_cycles = Param.Cycles(24, "Cycles for Keccak-p[1600, 24]")
    pad_cycles = Param.Cycles(1, "Cycles for padding")
