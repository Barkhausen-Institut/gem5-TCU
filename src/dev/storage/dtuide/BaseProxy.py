from m5.params import *
from m5.proxy import *
from MemObject import MemObject

class BaseProxy(MemObject):
    type = 'BaseProxy'
    cxx_header = 'dev/storage/dtuide/base_proxy.hh'

    dtu_port = SlavePort("Port to the DTU")
    dev_port = MasterPort("Port to the PCI setup")
    int_port = MasterPort("Port to send interrupts via GPU")

    system = Param.System(Parent.any, "System this tester is part of")
    max_data_size = Param.MemorySize("1kB", "The maximum size of data transfers")
    ranges = VectorParam.AddrRange("The corresponding address range")

    regfile_base_addr = Param.Addr(0xF0000000, "Register file address")
    id = Param.Unsigned("Core ID")
    size = Param.MemorySize("Dummy")
