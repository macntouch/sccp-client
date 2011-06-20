'''
Created on Jun 20, 2011

@author: lebleu1
'''
import unittest
from sccpphone import SCCPPhone
from mock import Mock
from sccp.sccpregister import SCCPRegister



class TestSCCPPhone(unittest.TestCase):
    
    
    def log(self,msg):
        print(msg)


    def testOnConnectSuccess(self):

        networkClient = Mock()
        sccpPhone = SCCPPhone('1.1.1.1','SEP001166554433')
        sccpPhone.setLogger(self.log)
        sccpPhone.client = networkClient
        sccpPhone.on_sccp_connect_success()
        registerMessage = SCCPRegister('SEP001166554433', "1.1.1.1")
        
        networkClient.sendSccpMessage.assert_called_with(registerMessage)


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()