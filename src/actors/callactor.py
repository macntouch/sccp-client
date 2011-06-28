'''
Created on Jun 28, 2011

@author: lebleu1
'''
from sccp.sccpcallstate import SCCPCallState


class CallActor():
    
    
    def setPhone(self,phone):
        self.phone = phone
        
                
    def handleCall(self,line,callid,callState):
        if callState == SCCPCallState.SCCP_CHANNELSTATE_RINGING:
            self.phone.answerCall()
