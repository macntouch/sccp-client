'''
Created on Jun 17, 2011

@author: lebleu1
'''

from PyQt4.QtCore import *
from PyQt4.QtGui import *

SKINNY_LBL_EMPTY = 0
SKINNY_LBL_REDIAL = 1
SKINNY_LBL_NEWCALL = 2
SKINNY_LBL_HOLD = 3
SKINNY_LBL_TRANSFER = 4
SKINNY_LBL_CFWDALL = 5
SKINNY_LBL_CFWDBUSY = 6
SKINNY_LBL_CFWDNOANSWER = 7
SKINNY_LBL_BACKSPACE = 8
SKINNY_LBL_ENDCALL = 9
SKINNY_LBL_RESUME = 10
SKINNY_LBL_ANSWER = 11

class SoftKeys(QVBoxLayout):

    def __init__(self, *args, **kwargs):
        QVBoxLayout.__init__(self, *args, **kwargs)
        self.createSoftKeyTexts()
        self.createSoftKeyButtons()
 
    def connectSoftKeys(self,softKeyHandler):
        self.softKeyHandler = softKeyHandler
       
    def createSoftKeyTexts(self):
        textBox = QHBoxLayout()
        textBox.setAlignment(Qt.AlignCenter)
        self.label1 = QLabel('NewCall')
        textBox.addWidget(self.label1)
        self.label2 = QLabel('EndCall')
        textBox.addWidget(self.label2)
        self.label3 = QLabel('Answer')
        textBox.addWidget(self.label3)
        self.label4 = QLabel('Redial')
        textBox.addWidget(self.label4)
        self.addLayout(textBox)
        
    def createSoftKeyButtons(self):
        buttonBox =QHBoxLayout()
        buttonBox.setAlignment(Qt.AlignCenter)
        self.createSoftKey(buttonBox,SKINNY_LBL_NEWCALL)
        self.createSoftKey(buttonBox,SKINNY_LBL_ENDCALL)
        self.createSoftKey(buttonBox,SKINNY_LBL_ANSWER)
        self.createSoftKey(buttonBox,SKINNY_LBL_REDIAL)
        self.addLayout(buttonBox)
                
    def createSoftKey(self,layout,content):
        key = QPushButton(str(content))
        key.setStyleSheet("background-color: #345677")
        key.clicked.connect(self.onSoftKey)
        layout.addWidget(key)
        
    def onSoftKey(self):
        self.softKeyHandler(int(str(self.sender().text())))
          


