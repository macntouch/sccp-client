'''
Created on Jun 20, 2011

@author: lebleu1
'''
from network.sccpclientfactory import SCCPClientFactory
from sccp.sccpmessagetype import SCCPMessageType
from sccp.sccpregister import SCCPRegister

class SCCPPhone():
    '''
    Main sccp phone class
    '''


    def __init__(self,host,deviceName):
        self.host = host
        self.deviceName = deviceName
        
    def setLogger(self,logger):
        self.log = logger
        
    def createClient(self):
        self.log('creating sccp client factory')
        self.client = SCCPClientFactory(
                        self.on_sccp_connect_success,
                        self.on_sccp_connect_fail)
        self.client.handleUnknownMessage(self.onUnknownMessage)
#        self.client.addHandler(SCCPMessageType.RegisterAckMessage,self.onRegisteredAck)
#        self.client.addHandler(SCCPMessageType.CapabilitiesReqMessage,self.onCapabilitiesReq)
#        self.client.addHandler(SCCPMessageType.KeepAliveAckMessage,self.onKeepAliveAck)
#        self.client.addHandler(SCCPMessageType.DefineTimeDate,self.onDefineTimeDate)
#        self.client.addHandler(SCCPMessageType.SetSpeakerModeMessage,self.onSetSpeakerMode)
#        self.client.addHandler(SCCPMessageType.CallStateMessage,self.onCallState)
#        self.client.addHandler(SCCPMessageType.ActivateCallPlaneMessage,self.onActivateCallPlane)
#        self.client.addHandler(SCCPMessageType.StartToneMessage,self.onStartTone)
        
        return self.client

    def on_sccp_connect_success(self):
        self.log('Connected to server. Sending register with phone set : ' + self.deviceName)
        registerMessage = SCCPRegister(self.deviceName, self.host)
        self.client.sendSccpMessage(registerMessage)
        
    def on_sccp_connect_fail(self, reason):
        # reason is a twisted.python.failure.Failure  object
        self.log('Connection failed: %s' % reason.getErrorMessage())

        