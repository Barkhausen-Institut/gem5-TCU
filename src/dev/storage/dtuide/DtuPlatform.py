from m5.params import *
from m5.proxy import *
from Platform import Platform
import BaseProxy

class DtuPlatform(Platform):
    type = 'DtuPlatform'
    idedtuproxy = Param.BaseProxy("Proxy serving as the interrupt deliverer")

    cxx_header = 'dev/storage/dtuide/dtu_platform.hh'
