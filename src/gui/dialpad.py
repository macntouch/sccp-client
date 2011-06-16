'''
Created on Jun 16, 2011

@author: lebleu1
'''

from PyQt4.QtCore import *
from PyQt4.QtGui import *


class DialPad(QVBoxLayout):
    '''
    classdocs
    '''

    def __init__(self, *args, **kwargs):
        QVBoxLayout.__init__(self, *args, **kwargs)
        self.createDialEditBox()
        self.createButtonRange(1)
        self.createButtonRange(4)
        self.createButtonRange(7)
        self.createLastRange()

   
    def createDialEditBox(self):
        editBox = QHBoxLayout()
        self.numberEdit = QLineEdit()
        editBox.addWidget(self.numberEdit)
        self.addLayout(editBox)
 
    def createButtonRange(self,start):
        buttonBox = QHBoxLayout()
        for i in range(3):
            self.createButton(buttonBox,str(i+start))
        self.addLayout(buttonBox)
        
    def createLastRange(self):
        buttonBox = QHBoxLayout()
        self.createButton(buttonBox, '*')
        self.createButton(buttonBox, '0')
        self.createButton(buttonBox, '#')
        self.addLayout(buttonBox)

        
    def createButton(self,layout,label):
        button = QPushButton(label);
        layout.addWidget(button)
        button.clicked.connect(self.onDialButton)

    def onDialButton(self):
        self.numberEdit.setText(str(self.numberEdit.text())+str(self.sender().text()))
