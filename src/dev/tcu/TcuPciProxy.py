# Copyright (c) 2017 Georg Kotheimer
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
from m5.params import *
from m5.proxy import *
from m5.objects.PciHost import *

class TcuPciHost(PciHost):
    type = 'TcuPciHost'
    cxx_header = "dev/tcu/pci_proxy.hh"

    pci_proxy = Param.TcuPciProxy("The PCI proxy")

    conf_base = Param.Addr("Config space base address")
    conf_size = Param.Addr(256 * 32 * 256, "Config space size")
    conf_device_bits = Param.UInt8(8, "Number of bits used as an "
                                      "offset to a devices address space")

    pci_pio_base = Param.Addr(0, "Base address for PCI IO accesses")
    pci_mem_base = Param.Addr(0, "Base address for PCI memory accesses")
    pci_dma_base = Param.Addr(0, "Base address for DMA memory accesses")


class TcuPciProxy(ClockedObject):
    type = 'TcuPciProxy'
    cxx_header = "dev/tcu/pci_proxy.hh"

    tcu_master_port = MasterPort("Port to send requests to the TCU")
    tcu_slave_port = SlavePort("Port that receives memory requests "
                                "from the TCU")
    pio_port = MasterPort("Port to the proxied device's pio port")
    dma_port = SlavePort("Port that receives DMA requests "
                         "from the proxied device")

    system = Param.System(Parent.any, "System the PCI proxy is part of")
    id = Param.Unsigned("Core ID")
    tcu_regfile_base_addr = Param.Addr(0xF0000000, "TCU register file address")
