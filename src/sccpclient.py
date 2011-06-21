""" 
Sample PyQt GUI using a Twisted-based socket client. Demonstrates how to make
PyQt work in unison with Twisted.

Eli Bendersky (eliben@gmail.com)
This code is in the public domain
"""
import sys, time
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from sccpphone import SCCPPhone
from gui.logwidget import LogWidget
from gui.phoneview import PhoneView

SERVER_HOST = '192.168.30.83'
SERVER_PORT = 2000
DEVICE_NAME= 'SEP00164697AAAA'

class SCCPClientWindow(QMainWindow):
    def __init__(self, reactor, parent=None):
        super(SCCPClientWindow, self).__init__(parent)
        self.reactor = reactor
        
        self.create_main_frame()
        self.create_timer()
        
 
    def create_main_frame(self):

        mainBox = QVBoxLayout()
        phoneBox = QHBoxLayout()
        self.mainPhoneView = PhoneView(SERVER_HOST,DEVICE_NAME,self.onConnect)
        phoneBox.addLayout(self.mainPhoneView)
        mainBox.addLayout(phoneBox)
        self.log_widget = LogWidget()
        mainBox.addWidget(self.log_widget)
       
        main_frame = QWidget()
        main_frame.setLayout(mainBox)
        main_frame.setMinimumWidth(300)
        
        self.setCentralWidget(main_frame)
        

    def create_timer(self):
        self.circle_timer = QTimer(self)
        self.circle_timer.timeout.connect(self.mainPhoneView.connectIndicator.next)
        self.circle_timer.start(25)
    
        
    def create_client(self,serverHost,deviceName):
        sccpPhone = SCCPPhone(serverHost,deviceName)
        sccpPhone.setLogger(self.log)
        sccpPhone.setTimerProvider(self)
        sccpPhone.setDateTimePicker(self.mainPhoneView)
        sccpPhone.setCallStateHandler(self.mainPhoneView)
        
        self.mainPhoneView.dialPad.connectPad(sccpPhone)
        self.mainPhoneView.softKeys.connectSoftKeys(sccpPhone.onSoftKey)

        sccpPhone.createClient()
        return sccpPhone     
        
    
    def onConnect(self,serverHost,deviceName):
        sccpPhone = self.create_client(serverHost,deviceName)
        self.log("trying to connect to : "+serverHost+ " on " +`SERVER_PORT`)
        self.connection = self.reactor.connectTCP(serverHost, SERVER_PORT, sccpPhone.client)

    
    def createTimer(self,intervalSecs,timerCallback):
        self.keepalive_timer = QTimer(self)
        self.keepalive_timer.timeout.connect(timerCallback)
        self.keepalive_timer.start(intervalSecs*1000)
        self.mainPhoneView.connectIndicator.connected = True
    
    def log(self, msg):
        timestamp = '[%010.3f]' % time.clock()
        self.log_widget.append(timestamp + ' ' + str(msg))

    def closeEvent(self, e):
        self.log("close event")
        self.reactor.stop()
        

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
