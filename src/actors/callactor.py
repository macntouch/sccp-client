'''
Created on Jun 28, 2011

@author: lebleu1
'''
from sccp.sccpcallstate import SCCPCallState
import random


class CallActor():
    
    callDurationMin = 2
    callDurationMax = 10
    
    def setPhone(self,phone):
        self.phone = phone
        
    def setTimerProvider(self,timerProvider):
        self.timerProvider = timerProvider
        
        
                
    def handleCall(self,line,callid,callState):
        if callState == SCCPCallState.SCCP_CHANNELSTATE_RINGING:
            self.phone.answerCall()
        if callState == SCCPCallState.SCCP_CHANNELSTATE_CONNECTED:
            timerInSec = random.randrange(self.callDurationMin,self.callDurationMax)
            self.timerProvider.createOneShotTimer(timerInSec,self.onCallEndTimer)
            
            
            
    def onCallEndTimer(self):
        self.phone.endCall()
