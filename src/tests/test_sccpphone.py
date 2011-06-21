'''
Created on Jun 20, 2011

@author: lebleu1
'''
from sccp.sccpbuttontemplatereq import SCCPButtonTemplateReq
from sccp.sccpcapabilities import SCCPCapabilitiesRes
from sccp.sccpcapabilitiesreq import SCCPCapabilitiesReq
from sccp.sccpregister import SCCPRegister
from sccp.sccpregisterack import SCCPRegisterAck
from sccp.sccpregisteravailablelines import SCCPRegisterAvailableLines
from sccp.sccptimedatereq import SCCPTimeDateReq
from sccpphone import SCCPPhone
from tests.mock import Mock
import unittest
from sccp.sccpdefinetimedate import SCCPDefineTimeDate
from sccp.sccpsetspeakermode import SCCPSetSpeakerMode
from sccp.sccpcallstate import SCCPCallState
from sccp.sccpkeypadbutton import SCCPKeyPadButton


        

class AnyInstanceOf(object):
    
    def __init__(self, clazz):
        self.clazz = clazz

    def __eq__(self,other):
        return self.clazz.__name__ == other.__class__.__name__
  
class TestSCCPPhone(unittest.TestCase):
    
    def log(self,msg):
        print(msg)

    def setUp(self):
        self.sccpPhone = SCCPPhone('1.1.1.1','SEP001166554433')
        self.sccpPhone.setLogger(self.log)


    def testOnConnectSuccess(self):

        networkClient = Mock()
        self.sccpPhone.client = networkClient
        self.sccpPhone.on_sccp_connect_success()
        registerMessage = SCCPRegister('SEP001166554433', "1.1.1.1")
        
        networkClient.sendSccpMessage.assert_called_with(registerMessage)


    def testOnRegisteredAck(self):
        timerProvider = Mock()
        registerAck = SCCPRegisterAck()
        registerAck.keepAliveInterval=25
        
        self.sccpPhone.setTimerProvider(timerProvider)
        self.sccpPhone.onRegisteredAck(registerAck)
        timerProvider.createTimer.assert_called_with(25,self.sccpPhone.onKeepAliveTimer)
        
    def testOnCapabilitesReq(self):
        networkClient = Mock()
        self.sccpPhone.client = networkClient

        self.sccpPhone.onCapabilitiesReq(SCCPCapabilitiesReq())
       
        networkClient.sendSccpMessage.assert_was_called_with(AnyInstanceOf(SCCPCapabilitiesRes))
        networkClient.sendSccpMessage.assert_was_called_with(AnyInstanceOf(SCCPButtonTemplateReq))
        networkClient.sendSccpMessage.assert_was_called_with(AnyInstanceOf(SCCPRegisterAvailableLines))
        networkClient.sendSccpMessage.assert_was_called_with(AnyInstanceOf(SCCPTimeDateReq))
    
    def testOnDefineTimeDate(self):
        defineDateTime = SCCPDefineTimeDate()
        defineDateTime.day=21
        defineDateTime.month=6
        defineDateTime.year=2011
        defineDateTime.hour=11
        defineDateTime.minute=40
        defineDateTime.seconds=36
        
        dateTimePicker = Mock()

        self.sccpPhone.setDateTimePicker(dateTimePicker)
        self.sccpPhone.onDefineTimeDate(defineDateTime)
        dateTimePicker.setDateTime.assert_called_with(21,6,2011,11,40,36)
        
    def testOnSetSpeakerMode(self):
        self.sccpPhone.onSetSpeakerMode(SCCPSetSpeakerMode())
        
    def testOnCallState(self):
        callStateHandler = Mock()
        self.sccpPhone.setCallStateHandler(callStateHandler)
        
        callState = SCCPCallState()
        callState.callId=43
        callState.line=2
        callState.callState=SCCPCallState.SCCP_CHANNELSTATE_RINGING
        
        self.sccpPhone.onCallState(callState)
        
        callStateHandler.handleCall.assert_called_with(2,43,SCCPCallState.SCCP_CHANNELSTATE_RINGING)
    
    def testOnDialNumericButtonPushed(self):
        networkClient = Mock()
        self.sccpPhone.client = networkClient
        dialPadMessage = SCCPKeyPadButton(int('1'))
        self.sccpPhone.onDialButtonPushed('1')
    
        networkClient.sendSccpMessage.assert_called_with(dialPadMessage)
         
    def testOnDialHashButtonPushed(self):
        networkClient = Mock()
        self.sccpPhone.client = networkClient
        dialPadMessage = SCCPKeyPadButton(15)
        self.sccpPhone.onDialButtonPushed('#')
    
        networkClient.sendSccpMessage.assert_called_with(dialPadMessage)

    def testOnDialStarButtonPushed(self):
        networkClient = Mock()
        self.sccpPhone.client = networkClient
        dialPadMessage = SCCPKeyPadButton(14)
        self.sccpPhone.onDialButtonPushed('*')
    
        networkClient.sendSccpMessage.assert_called_with(dialPadMessage)

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()