""" 
Sample PyQt GUI using a Twisted-based socket client. Demonstrates how to make
PyQt work in unison with Twisted.

Eli Bendersky (eliben@gmail.com)
This code is in the public domain
"""
import sys, time
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from struct import pack

from sccp.sccpmessage import SCCPMessage
from sccp.sccpmessagetype import SCCPMessageType
from gui.connectindicator import ConnectIndicator
from gui.dialpad import DialPad
from gui.softkeys import SoftKeys
from sccp.sccpsoftkeyevent import SCCPSoftKeyEvent
from sccp.sccpkeypadbutton import SCCPKeyPadButton
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
        #self.create_client()
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
        self.sccpPhone.setDateTimePicker(self)
        self.sccpPhone.setCallStateHandler(self)
        self.client = self.sccpPhone.createClient()        
        
    
    def on_doit(self):
        self.create_client()
        serverHost = str(self.hostEdit.text())
        self.log("trying to connect to : "+serverHost+ " on " +`SERVER_PORT`)
        self.connection = self.reactor.connectTCP(serverHost, SERVER_PORT, self.client)

    
    def createTimer(self,intervalSecs,timerCallback):
        self.keepalive_timer = QTimer(self)
        self.keepalive_timer.timeout.connect(timerCallback)
        self.keepalive_timer.start(intervalSecs*1000)
        self.circle_widget.connected = True
        
      
    def setDateTime(self,day,month,year,hour,minute,seconds):
        self.timeDateLabel.setText(`day` + '-'+`month` + '-' + `year` 
                                   + ' ' +`hour`+':'+`minute`+':'+`seconds`)

        
    def handleCall(self,line,callId,callState):
        self.callDisplay.displayCall(line, callId, callState)
        self.currentLine = line
        self.currentCallId=callId
        self.callState=callState
                  
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
        if (event != 2):
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
