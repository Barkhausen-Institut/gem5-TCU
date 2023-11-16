# Copyright (C) 2022 Nils Asmussen, Barkhausen Institut
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
from m5.objects.System import System

class M3System(System):
    type = 'M3System'
    cxx_header = 'sim/m3_system.hh'
    cxx_class = 'gem5::M3System'

    cmdline = Param.String("", "The command line arguments")
    logflags = Param.String("", "The log flags (LOG)")

    tile_id = Param.Unsigned("The tile id")

    memory_tile = Param.Unsigned(0, "The memory tile to use")
    memory_offset = Param.Addr(0, "The offset in the memory tile")
    memory_size = Param.Addr(0, "The size in the memory tile")

    env_start = Param.Addr(0x100000, "The environment start address")

    mod_offset = Param.Addr(0, "The offset of the boot modules (only for kernel tile)")
    mod_size = Param.Addr(0, "The max. size of the boot modules (only for kernel tile)")
    tile_size = Param.Addr(0, "The size of the reserved memory region for each tile (only for kernel tile)")

    internal_eps = Param.Bool(False, "True if the TCU has internal endpoints")
    eps_addr = Param.Addr(0, "The address of the endpoint region")
    eps_num = Param.Addr(0, "The number of endpoints")

    mods = VectorParam.String([], "The boot modules")
    tile_descs = VectorParam.Addr([], "All tile descriptions with their type and mem size")
    tile_ids = VectorParam.Addr([], "All tile ids")

    noc_master_port = RequestPort("Port that connects to the global NoC (only for initialization)")

    rot_layers = VectorParam.String([], "The RoT layers (excluding the Boot ROM)")
    kernel_cmdline = Param.String("", "The command line arguments for the kernel")
