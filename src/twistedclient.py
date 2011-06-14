""" 
Simple socket client using Twisted.

Eli Bendersky (eliben@gmail.com)
This code is in the public domain
"""
import struct

from twisted.internet.protocol import Protocol, ClientFactory
from twisted.protocols.basic import IntNStringReceiver
from sccp.sccpmessagetype import SCCPMessageType


class SocketClientProtocol(IntNStringReceiver):
    """ The protocol is based on twisted.protocols.basic
        IntNStringReceiver, with little-endian 32-bit 
        length prefix.
    """
    structFormat = "<L"
    prefixLength = struct.calcsize(structFormat)
    trailingNbOfBytes = 4
    
    def dataReceived(self, recd):
        """
        Convert int prefixed strings into calls to stringReceived.
        """
        print "data received"
        self.recvd = self.recvd + recd
        while len(self.recvd) >= self.prefixLength and not self.paused:
            length ,= struct.unpack(
                self.structFormat, self.recvd[:self.prefixLength])
            length=length+self.trailingNbOfBytes
            
            if length > self.MAX_LENGTH:
                self.lengthLimitExceeded(length)
                return
            if len(self.recvd) < length + self.prefixLength:
                break
            packet = self.recvd[self.prefixLength:length + self.prefixLength]
            self.recvd = self.recvd[length + self.prefixLength:]
            self.stringReceived(packet)

    def stringReceived(self, s):    
        self.factory.got_msg(s);

        #self.factory.onMessage(messageType)
#        if messageType == SCCPMessageType.RegisterAckMessage:
#            self.factory.onRegisterAck()

    def connectionMade(self):
        self.factory.clientReady(self)
    
        
    def sendString(self, data):
        print "sending " +str(len(data))
#IntNStringReceiver.sendString(self, data)
        self.transport.write(struct.pack(self.structFormat, len(data)) + data+"\x00\x00\x00\x00")



class SocketClientFactory(ClientFactory):
    """ Created with callbacks for connection and receiving.
        send_msg can be used to send messages when connected.
    """
    protocol = SocketClientProtocol

    def __init__(   
            self, 
            connect_success_callback,
            connect_fail_callback,
            recv_callback):
        self.connect_success_callback = connect_success_callback
        self.connect_fail_callback = connect_fail_callback
        self.recv_callback = recv_callback
        self.client = None
    
    def clientConnectionFailed(self, connector, reason):
        self.connect_fail_callback(reason)
    
    def clientReady(self, client):
        self.client = client
        self.connect_success_callback()
    
    def got_msg(self, msg):
        self.recv_callback(msg)
        
    def send_msg(self, msg):
        if self.client:
            self.client.sendString(msg)
            
    def clientConnectionLost(self, connector, reason):
        print 'Lost connection.  Reason:', reason
            


