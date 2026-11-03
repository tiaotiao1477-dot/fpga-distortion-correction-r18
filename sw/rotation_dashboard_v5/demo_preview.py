"""Generated UI smoke source. Never opens a device."""
import time
import numpy as np
class DemoPreview:
    def __init__(self):
        self.running=False;self.started=0
        y,x=np.indices((540,960));self.frame=np.empty((540,960,3),dtype=np.uint8)
        self.frame[:,:,0]=(100+x*90/960).astype(np.uint8)
        self.frame[:,:,1]=(170+y*50/540).astype(np.uint8)
        self.frame[:,:,2]=(200-x*40/960).astype(np.uint8)
        self.frame[((x//80+y//80)%2)==0] //= 2
    def start(self):self.running=True;self.started=time.monotonic()
    def stop(self):self.running=False
    def status(self):return {'state':'running' if self.running else 'stopped','fps':30.0 if self.running else 0.0,'process_alive':False,'reader_alive':False,'demo':True}
    def latest_with_identity(self):
        if not self.running:return None
        now=time.monotonic();return(1,int((now-self.started)*30),now,self.frame)
    def latest(self):return self.frame if self.running else None
