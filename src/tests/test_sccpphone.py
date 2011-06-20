'''
Created on Jun 20, 2011

@author: lebleu1
'''
import unittest
from sccpphone import SCCPPhone
from mock import Mock
from sccp.sccpregister import SCCPRegister
from sccp.sccpregisterack import SCCPRegisterAck



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
        
        
        
if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()