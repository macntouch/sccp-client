'''
Created on Jun 14, 2011

@author: lebleu1
'''
from struct import pack
class IpAddress:
    
    def __init__(self,address):
        self.address = address
        
        
    def pack(self):
        bytes=self.address.split(".")
        strPack = ""
        for byte in bytes:
            strPack = strPack + pack("B",int(byte))
        return strPack
        