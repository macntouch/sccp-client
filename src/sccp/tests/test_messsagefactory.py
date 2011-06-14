'''
Created on Jun 14, 2011

@author: lebleu1
'''
import unittest
from sccp.messagefactory import MessageFactory
from sccp.sccpregisterack import SCCPRegisterAck


class TestMessageFactory(unittest.TestCase):

    def setUp(self):
        self.messageFactory = MessageFactory()

    def testCreateRegisterAck(self):
        receivedBuffer = "\x00\x00\x00\x00\x81\x00\x00\x00\x00\x0b\x00\x00"
        
        msg = self.messageFactory.create(receivedBuffer)
        
        self.assertTrue(isinstance(msg, SCCPRegisterAck))
        
    def testCreateUnkownType(self):
        receivedBuffer = "\x00\x00\x00\x00\xFF\xFF\x00\x00\x00\x0b\x00\x00"
        msg = self.messageFactory.create(receivedBuffer)
        self.assertEquals(0xFFFF,msg.sccpmessageType)
        


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()