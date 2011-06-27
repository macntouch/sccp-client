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
DEVICE_NAME1= 'SEP00164697AAAA'
DEVICE_NAME2= 'SEP00164697AAAB'

class SCCPClientWindow(QMainWindow):
    def __init__(self, reactor, parent=None):
        super(SCCPClientWindow, self).__init__(parent)
        self.phoneViews=[]
        self.reactor = reactor
        
        self.create_main_frame()
        self.createPhones()
        self.createIndicatorTimer()
        
        oneTimer = QTimer(self)
        oneTimer.setSingleShot(True)
        oneTimer.timeout.connect(self.ontimedoit)
        oneTimer.start(5000)
    def ontimedoit(self):
        self.log('I did it')
 
    def create_main_frame(self):

        mainBox = QVBoxLayout()
        self.phoneBox = QHBoxLayout()
        mainBox.addLayout(self.phoneBox)
        self.log_widget = LogWidget()
        mainBox.addWidget(self.log_widget)
       
        main_frame = QWidget()
        main_frame.setLayout(mainBox)
        main_frame.setMinimumWidth(300)
        
        self.setCentralWidget(main_frame)
        
    def createPhones(self):
        mainPhoneView = PhoneView(SERVER_HOST,DEVICE_NAME1,self.onConnect)
        self.phoneBox.addLayout(mainPhoneView)
        sccpPhone = SCCPPhone(SERVER_HOST,DEVICE_NAME1)
        sccpPhone.setLogger(self.log)
        sccpPhone.setTimerProvider(self)
        mainPhoneView.useSccpPhone(sccpPhone)
        self.phoneViews.append(mainPhoneView)
        
        mainPhoneView = PhoneView(SERVER_HOST,DEVICE_NAME2,self.onConnect)
        self.phoneBox.addLayout(mainPhoneView)
        sccpPhone = SCCPPhone(SERVER_HOST,DEVICE_NAME2)
        sccpPhone.setLogger(self.log)
        sccpPhone.setTimerProvider(self)
        mainPhoneView.useSccpPhone(sccpPhone)
        self.phoneViews.append(mainPhoneView)
        
    def createIndicatorTimer(self):
        self.circle_timer = QTimer(self)
        self.circle_timer.timeout.connect(self.onIndicatorTimer)
        self.circle_timer.start(25)
        
    def onIndicatorTimer(self):
        for phoneView in self.phoneViews:
            phoneView.connectIndicator.next()
            
    def onConnect(self,serverHost,deviceName,networkClient):
        self.log("trying to connect to : "+serverHost+ " on " +`SERVER_PORT`)
        self.connection = self.reactor.connectTCP(serverHost, SERVER_PORT, networkClient)

    
    def createTimer(self,intervalSecs,timerCallback):
        self.keepalive_timer = QTimer(self)
        self.keepalive_timer.timeout.connect(timerCallback)
        self.keepalive_timer.start(intervalSecs*1000)
    
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
