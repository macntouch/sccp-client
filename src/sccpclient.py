""" 
Sample PyQt GUI using a Twisted-based socket client. Demonstrates how to make
PyQt work in unison with Twisted.

Eli Bendersky (eliben@gmail.com)
This code is in the public domain
"""
import os, sys, time
import Queue
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from struct import pack

from sccp.sccpmessage import SCCPMessage
from sccp.sccpmessagetype import SCCPMessageType
import struct
from sccp.sccpregister import SCCPRegister
from network.sccpclientfactory import SCCPClientFactory
from sccp.sccpcapabilities import SCCPCapabilitiesRes
from sccp.sccpregisteravailablelines import SCCPRegisterAvailableLines
from gui.connectindicator import ConnectIndicator
from gui.dialpad import DialPad
from gui.softkeys import SoftKeys
from sccp.sccpsoftkeyevent import SCCPSoftKeyEvent
from sccp.sccpkeypadbutton import SCCPKeyPadButton
from sccp.sccpcallstate import SCCPCallState
from gui.calldisplay import CallDisplay
from sccpphone import SCCPPhone

SERVER_HOST = '192.168.30.83'
SERVER_PORT = 2000
DEVICE_NAME= 'SEP00164697AAAA'

class LogWidget(QTextBrowser):
    def __init__(self, parent=None):
        super(LogWidget, self).__init__(parent)
        palette = QPalette()
        palette.setColor(QPalette.Base, QColor("#ddddfd"))
        self.setPalette(palette)
    def minimumSizeHint(self, *args, **kwargs):
        return QSize(400,300)


class SCCPClientWindow(QMainWindow):
    def __init__(self, reactor, parent=None):
        super(SCCPClientWindow, self).__init__(parent)
        self.reactor = reactor
        
        self.create_main_frame()
        self.create_client()
        self.create_timer()
        
        self.line = 0
        self.callId = 0

    def create_time_box(self):
        timeBox = QHBoxLayout()
        self.timeDateLabel = QLabel('..................')
        timeBox.addWidget(self.timeDateLabel)
        return timeBox        

    def create_main_frame(self):

        self.circle_widget = ConnectIndicator()
        self.doit_button = QPushButton('Connect !')
        self.doit_button.clicked.connect(self.on_doit)
        self.log_widget = LogWidget()
        self.hostEdit = QLineEdit()
        self.hostEdit.setText(SERVER_HOST)
        self.deviceNameEdit = QLineEdit()
        self.deviceNameEdit.setText(DEVICE_NAME)
        
        hbox = QVBoxLayout()
        hbox.addLayout(self.create_time_box())
        hbox.addWidget(self.circle_widget)
        self.callDisplay = CallDisplay()
        hbox.addLayout(self.callDisplay)
        softKeys = SoftKeys()
        softKeys.connectSoftKeys(self.onSoftKey)
        hbox.addLayout(softKeys)
        
        dialPad = DialPad()
        dialPad.connectPad(self.onDialButtonPushed)
        hbox.addLayout(dialPad)
        
        self.createDeviceParameters(hbox)

        hbox.addWidget(self.doit_button)
        hbox.addWidget(self.log_widget)

                
        main_frame = QWidget()
        main_frame.setLayout(hbox)
        main_frame.setMinimumWidth(400)
        
        self.setCentralWidget(main_frame)
        
    def createDeviceParameters(self,layout):
        hostLabel = QLabel("Host : ")
        hostBox = QHBoxLayout()
        hostBox.addWidget(hostLabel)
        hostBox.addWidget(self.hostEdit)
        deviceNameLabel = QLabel("Phone Set : ")
        hostBox.addWidget(deviceNameLabel)
        hostBox.addWidget(self.deviceNameEdit)
        layout.addLayout(hostBox)

    def create_timer(self):
        self.circle_timer = QTimer(self)
        self.circle_timer.timeout.connect(self.circle_widget.next)
        self.circle_timer.start(25)
    
    def create_keepalive_timer(self):
        self.circle_timer = QTimer(self)
        self.circle_timer.timeout.connect(self.circle_widget.next)
        self.circle_timer.start(25)
        
    def create_client(self):
        serverHost = str(self.hostEdit.text())
        deviceName = str(self.deviceNameEdit.text())
        self.sccpPhone = SCCPPhone(serverHost,deviceName)
        self.sccpPhone.setLogger(self.log)
        self.sccpPhone.setTimerProvider(self)
        self.client = self.sccpPhone.createClient()        
        
    
    def on_doit(self):
       
        serverHost = str(self.hostEdit.text())
        self.log("trying to connect to : "+serverHost+ " on " +`SERVER_PORT`)
        self.connection = self.reactor.connectTCP(serverHost, SERVER_PORT, self.client)

    
    def createTimer(self,intervalSecs,timerCallback):
        self.keepalive_timer = QTimer(self)
        self.keepalive_timer.timeout.connect(timerCallback)
        self.keepalive_timer.start(intervalSecs*1000)
        self.circle_widget.connected = True
        
    
    def onCapabilitiesReq(self,message):
        self.log("On capabilities request")
        self.log("sending capabilities response")
        capabilities = SCCPCapabilitiesRes()
        self.client.send_msg(capabilities.pack())
        self.log("sending button template request message")
        message = SCCPMessage(SCCPMessageType.ButtonTemplateReqMessage)
        self.client.send_msg(message.pack())        
        self.log("sending register available lines")
        message=SCCPRegisterAvailableLines()
        self.client.send_msg(message.pack())
        self.log("sending time date request message")
        message = SCCPMessage(SCCPMessageType.TimeDateReqMessage)
        self.client.send_msg(message.pack())
        
  
    def onDefineTimeDate(self,message):
        self.log('define time and date')
        self.timeDateLabel.setText(`message.day` + '-'+`message.month` + '-' + `message.year` 
                                   + ' ' +`message.hour`+':'+`message.minute`+':'+`message.seconds`)

    def onSetSpeakerMode(self,message):
        self.log('set speaker mode '+`message.mode`)
        
    def onCallState(self,message):
        self.log('call state line : ' + `message.line` + ' for callId '+ `message.callId` + ' is ' + SCCPCallState.sccp_channelstates[message.callState])
        self.callDisplay.displayCall(message.line, message.callId, message.callState)
        self.currentLine = message.line
        self.currentCallId=message.callId
        self.callState=message.callState
        
    def onActivateCallPlane(self,message):
        self.log('Activate call plane on line '+`message.line`)
    
    def onStartTone(self,message):
        self.log('start tone : '+`message.tone` + ' timeout ' + `message.toneTimeout` + ' line ' + `message.line` + ' for callId '+ `message.callId`)
        
    def sendKeepAlive(self):
        self.log("sending keepalive")
        message = SCCPMessage(SCCPMessageType.KeepAliveMessage)
        strMessage = message.pack();
        self.client.send_msg(strMessage)
                
    def log(self, msg):
        timestamp = '[%010.3f]' % time.clock()
        self.log_widget.append(timestamp + ' ' + str(msg))

    def closeEvent(self, e):
        self.log("close event")
        self.reactor.stop()

    def onDialButtonPushed(self,car):
        self.log("dialed : " + car)
        message = SCCPKeyPadButton(int(car))
        self.client.send_msg(message.pack())
        
    def onSoftKey(self,event):
        self.log('on soft key '+`event`)
        if (event == 9):
            message = SCCPSoftKeyEvent(event,self.currentLine,self.currentCallId)
        else:
            message = SCCPSoftKeyEvent(event)
        self.client.send_msg(message.pack())

#-------------------------------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    try:
        import qt4reactor
    except ImportError:
        # Maybe qt4reactor is placed inside twisted.internet in site-packages?
        from twisted.internet import qt4reactor
    qt4reactor.install()
    
    from twisted.internet import reactor
    mainwindow = SCCPClientWindow(reactor)
    mainwindow.show()
    
    reactor.run()
