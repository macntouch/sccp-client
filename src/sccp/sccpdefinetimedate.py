'''
Created on Jun 14, 2011

@author: lebleu1
'''
from sccp.sccpmessage import SCCPMessage
from sccp.sccpmessagetype import SCCPMessageType
from struct import unpack


class SCCPDefineTimeDate(SCCPMessage):
    

    def __init__(self):
        SCCPMessage.__init__(self, SCCPMessageType.DefineTimeDate)
        self.year=0
        self.month=0
        self.dayOfWeek=0
        self.day=0
        self.hour=99
        self.minute=99
        self.seconds=99
        self.milliseconds=99
        self.timestamp=0
        

    def unPack(self,buffer):
        self.year = unpack("L",buffer[:4])[0]
        self.month = unpack("L",buffer[4:8])[0]
        self.dayOfWeek = unpack("L",buffer[8:12])[0]
        self.day = unpack("L",buffer[12:16])[0]
        self.hour=unpack("L",buffer[16:20])[0]
        self.minute=unpack("L",buffer[20:24])[0]
        self.seconds=unpack("L",buffer[24:28])[0]
        self.milliseconds=unpack("L",buffer[28:32])[0]
        self.timestamp=unpack("L",buffer[32:36])[0]
