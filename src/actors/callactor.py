'''
Created on Jun 28, 2011

@author: lebleu1
'''


class CallActor():
    
    
    def setPhone(self,phone):
        self.phone = phone
        
        
    def onNewCall(self):
        self.phone.answerCall()