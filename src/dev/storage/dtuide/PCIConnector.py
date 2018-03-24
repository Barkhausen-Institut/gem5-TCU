from m5.params import *
from m5.proxy import *
from Connector import BaseConnector

class PCIConnector(BaseConnector):
    cxx_header = 'dev/storage/dtuide/pci_connector.hh'
    type = 'PCIConnector'

    base_proxy = Param.BaseProxy('Proxy to be connected to')
