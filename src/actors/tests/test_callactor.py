'''
Created on Jun 28, 2011

@author: lebleu1
'''
import unittest
from actors.callactor import CallActor
from tests.mock import Mock


class Test(unittest.TestCase):
    
    
    def setUp(self):
        self.callActor = CallActor()
        self.sccpPhone = Mock()
        self.callActor.setPhone(self.sccpPhone)


    def testOnNewCall(self):
        
        self.callActor.onNewCall()
        
        self.sccpPhone.answerCall.assert_called_with()


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()