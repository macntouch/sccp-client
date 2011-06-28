'''
Created on Jun 28, 2011

@author: lebleu1
'''
import unittest
from actors.callactor import CallActor
from tests.mock import Mock
from sccp.sccpcallstate import SCCPCallState


class Test(unittest.TestCase):
    
    
    def setUp(self):
        self.callActor = CallActor()
        self.sccpPhone = Mock()
        self.callActor.setPhone(self.sccpPhone)

        
    def testOnCallRinging(self):
        self.callActor.handleCall(1,34,SCCPCallState.SCCP_CHANNELSTATE_RINGING)
        self.callActor.handleCall(1,34,SCCPCallState.SCCP_CHANNELSTATE_OFFHOOK)
        self.sccpPhone.answerCall.assert_called_once_with()


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()