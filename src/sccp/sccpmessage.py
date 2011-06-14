'''
Created on Jun 10, 2011

@author: lebleu1
'''
import struct

class SCCPMessage:
    '''
    Sccp message
    '''

    def __init__(self,sccpMessageType):
        self.sccpmessageType = sccpMessageType
        self.reserved=0x00
        
    def pack(self):
        return struct.pack("LL",self.reserved,self.sccpmessageType)
    
    def unPack(self,buffer):
        self.buffer = buffer
        
    def toStr(self):
        return "SCCPMessage : " + hex(self.sccpmessageType)