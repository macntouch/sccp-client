'''
Created on Jun 14, 2011

@author: lebleu1
'''
from sccp.sccpmessage import SCCPMessage
from sccp.sccpmessagetype import SCCPMessageType
from struct import pack

class SCCPSoftKeyEvent(SCCPMessage):
    '''
    sccp register message
    '''
    def __init__(self,event,line=0,callId=0):
        '''
        Constructor
        '''
        SCCPMessage.__init__(self, SCCPMessageType.SoftKeyEventMessage)
        self.event=event
        self.line=line
        self.callId=callId
        
    def pack(self):
        strPack = SCCPMessage.pack(self)
        strPack = strPack + pack("L",self.event)
        strPack = strPack + pack("L",self.line)
        strPack = strPack + pack("L",self.callId)
        return strPack
