'''
Created on Jun 20, 2011

@author: lebleu1
'''
from network.sccpclientfactory import SCCPClientFactory
from sccp.sccpmessagetype import SCCPMessageType
from sccp.sccpregister import SCCPRegister
from sccp.sccpcapabilities import SCCPCapabilitiesRes
from sccp.sccpbuttontemplatereq import SCCPButtonTemplateReq
from sccp.sccpregisteravailablelines import SCCPRegisterAvailableLines
from sccp.sccptimedatereq import SCCPTimeDateReq
from sccp.sccpcallstate import SCCPCallState

class SCCPPhone():
    '''
    Main sccp phone class
    '''


    def __init__(self,host,deviceName):
        self.host = host
        self.deviceName = deviceName
        
    def setLogger(self,logger):
        self.log = logger
    def setTimerProvider(self,timerProvider):
        self.timerProvider = timerProvider
        
    def setDateTimePicker(self,dateTimePicker):
        self.dateTimePicker = dateTimePicker
        
    def setCallStateHandler(self,callStateHandler):
        self.callStateHandler=callStateHandler
        
    def createClient(self):
        self.log('creating sccp client factory')
        self.client = SCCPClientFactory(
                        self.on_sccp_connect_success,
                        self.on_sccp_connect_fail)
        self.client.handleUnknownMessage(self.onUnknownMessage)
        self.client.addHandler(SCCPMessageType.RegisterAckMessage,self.onRegisteredAck)
        self.client.addHandler(SCCPMessageType.CapabilitiesReqMessage,self.onCapabilitiesReq)
        self.client.addHandler(SCCPMessageType.KeepAliveAckMessage,self.onKeepAliveAck)
        self.client.addHandler(SCCPMessageType.DefineTimeDate,self.onDefineTimeDate)
        self.client.addHandler(SCCPMessageType.SetSpeakerModeMessage,self.onSetSpeakerMode)
        self.client.addHandler(SCCPMessageType.CallStateMessage,self.onCallState)
        self.client.addHandler(SCCPMessageType.ActivateCallPlaneMessage,self.onActivateCallPlane)
        self.client.addHandler(SCCPMessageType.StartToneMessage,self.onStartTone)
        
        return self.client

    def on_sccp_connect_success(self):
        self.log('Connected to server. Sending register with phone set : ' + self.deviceName)
        registerMessage = SCCPRegister(self.deviceName, self.host)
        self.client.sendSccpMessage(registerMessage)
        
    def on_sccp_connect_fail(self, reason):
        # reason is a twisted.python.failure.Failure  object
        self.log('Connection failed: %s' % reason.getErrorMessage())
        
    def onKeepAliveTimer(self):
        self.log('on keep alive')
        
    def onUnknownMessage(self,message):
        self.log('receive unkown message ' + message.toStr())

    def onRegisteredAck(self,registerAck):
        self.log("sccp phone registered")
        self.log("--          keepAliveInterval : " + `registerAck.keepAliveInterval`)
        self.log("--               dateTemplate : " + `registerAck.dateTemplate`)
        self.log("-- secondaryKeepAliveInterval : " + `registerAck.secondaryKeepAliveInterval`)
        self.timerProvider.createTimer(registerAck.keepAliveInterval,self.onKeepAliveTimer)

        
    def onKeepAliveAck(self,message):
        self.log("Keepalive ack")
    
    def onCapabilitiesReq(self,message):
        self.log("sending capabilities response")
        self.client.sendSccpMessage(SCCPCapabilitiesRes())
        self.log("sending button template request message")
        self.client.sendSccpMessage(SCCPButtonTemplateReq())
        self.log("sending register available lines")
        self.client.sendSccpMessage(SCCPRegisterAvailableLines())
        self.log("sending time date request message")
        self.client.sendSccpMessage(SCCPTimeDateReq())

        
    def onDefineTimeDate(self,message):
        self.log('define time and date')
        self.dateTimePicker.setDateTime(message.day,message.month,message.year,message.hour,message.minute,message.seconds)
    
    def onSetSpeakerMode(self,message):
        self.log('set speaker mode '+`message.mode`)

    def onCallState(self,message):
        self.log('call state line : ' + `message.line` + ' for callId '+ `message.callId` + ' is ' + SCCPCallState.sccp_channelstates[message.callState])
        self.callStateHandler.handleCall(message.line,message.callId,message.callState)
#        self.callDisplay.displayCall(message.line, message.callId, message.callState)
#        self.currentLine = message.line
#        self.currentCallId=message.callId
#        self.callState=message.callState

    def onStartTone(self,message):
        self.log('start tone : '+`message.tone` + ' timeout ' + `message.toneTimeout` + ' line ' + `message.line` + ' for callId '+ `message.callId`)

    def onActivateCallPlane(self,message):
        self.log('Activate call plane on line '+`message.line`)
