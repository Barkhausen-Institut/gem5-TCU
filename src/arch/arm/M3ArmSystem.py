# Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
# Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# The views and conclusions contained in the software and documentation are
# those of the authors and should not be interpreted as representing official
# policies, either expressed or implied, of the FreeBSD Project.

from m5.params import *
from m5.objects.ArmSystem import ArmSystem

class M3ArmSystem(ArmSystem):
    type = 'M3ArmSystem'
    cxx_header = 'arch/arm/m3/system.hh'

    cmdline = Param.String("", "The command line arguments")

    tile_id = Param.Unsigned("The tile id")

    memory_tile = Param.Unsigned(0, "The memory tile to use")
    memory_offset = Param.Addr(0, "The offset in the memory tile")
    memory_size = Param.Addr(0, "The size in the memory tile")

    mod_offset = Param.Addr(0, "The offset of the boot modules (only for kernel tile)")
    mod_size = Param.Addr(0, "The max. size of the boot modules (only for kernel tile)")
    tile_size = Param.Addr(0, "The size of the reserved memory region for each tile (only for kernel tile)")

    mods = VectorParam.String([], "The boot modules")
    tiles = VectorParam.Addr([], "All tiles in the system with their type and mem size")

    noc_master_port = MasterPort("Port that connects to the global NoC (only for initialization)")
