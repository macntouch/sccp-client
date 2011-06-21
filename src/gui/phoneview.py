'''
Created on Jun 21, 2011

@author: lebleu1
'''
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from gui.connectindicator import ConnectIndicator
from gui.calldisplay import CallDisplay
from gui.softkeys import SoftKeys
from gui.dialpad import DialPad

class PhoneView(QVBoxLayout):
    '''
    classdocs
    '''


    def __init__(self,serverHost,deviceName,connectHandler):
        QVBoxLayout.__init__(self)
        self.serverHost = serverHost
        self.deviceName = deviceName
        self.connectHandler=connectHandler
        self.createView()
 
 
    def createView(self):
        self.connectIndicator = ConnectIndicator()
        self.addWidget(self.connectIndicator)
        self.create_time_box()
        
        self.callDisplay = CallDisplay()
        self.addLayout(self.callDisplay)

        self.softKeys = SoftKeys()
        self.addLayout(self.softKeys)

        self.dialPad = DialPad()
        self.addLayout(self.dialPad)

        self.createDeviceParameters()
        
        self.connectButton = QPushButton('Connect !')
        self.connectButton.clicked.connect(self.onConnect)
        self.addWidget(self.connectButton)
 
        
        
    def create_time_box(self):
        timeBox = QHBoxLayout()
        self.timeDateLabel = QLabel('..................')
        timeBox.addWidget(self.timeDateLabel)
        self.addLayout(timeBox)        
        
        
    def createDeviceParameters(self):
        self.hostEdit = QLineEdit()
        self.hostEdit.setText(self.serverHost)
        self.deviceNameEdit = QLineEdit()
        self.deviceNameEdit.setText(self.deviceName)
        hostLabel = QLabel("Host : ")
        hostBox = QHBoxLayout()
        hostBox.addWidget(hostLabel)
        hostBox.addWidget(self.hostEdit)
        deviceNameLabel = QLabel("Phone Set : ")
        hostBox.addWidget(deviceNameLabel)
        hostBox.addWidget(self.deviceNameEdit)
        self.addLayout(hostBox)
        
    def onConnect(self):
        self.connectHandler(str(self.hostEdit.text()),str(self.deviceNameEdit.text()))
        
    def setDateTime(self,day,month,year,hour,minute,seconds):
        self.timeDateLabel.setText(`day` + '-'+`month` + '-' + `year` 
                                   + ' ' +`hour`+':'+`minute`+':'+`seconds`)
        
    def handleCall(self,line,callId,callState):
        self.callDisplay.displayCall(line, callId, callState)
         
        