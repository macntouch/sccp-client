'''
Created on Jun 14, 2011

@author: lebleu1
'''

from struct import unpack
from sccp.sccpmessagetype import SCCPMessageType
from sccp.sccpregisterack import SCCPRegisterAck
from sccp.sccpmessage import SCCPMessage
from sccp.sccpcapabilitiesreq import SCCPCapabilitiesReq
from sccp.sccpkeepaliveack import SCCPKeepAliveAck
from sccp.sccpdefinetimedate import SCCPDefineTimeDate
from sccp.sccpsetspeakermode import SCCPSetSpeakerMode
from sccp.sccpcallstate import SCCPCallState

class MessageFactory():
    '''
    sccp message factory create message from received buffer
    '''
    def __init__(self):
        '''
        '''
    def create(self,buffer):
        messageType = unpack("L",buffer[4:8])[0]
        msg = SCCPMessage(messageType)
        if (messageType == SCCPMessageType.RegisterAckMessage):
            msg = SCCPRegisterAck()
        if (messageType == SCCPMessageType.CapabilitiesReqMessage):
            msg = SCCPCapabilitiesReq()
        if (messageType == SCCPMessageType.KeepAliveAckMessage):
            msg = SCCPKeepAliveAck()
        if (messageType ==  SCCPMessageType.DefineTimeDate):
            msg = SCCPDefineTimeDate()
        if (messageType == SCCPMessageType.SetSpeakerModeMessage):
            msg = SCCPSetSpeakerMode()
        if (messageType == SCCPMessageType.CallStateMessage):
            msg = SCCPCallState()
        return msg