
import os
import time
import multiprocessing.dummy as mp
import rospy
from rosgraph_msgs.msg import Clock


class TimerProcess :
    
    def __init__ (self, _startTimestamp=0, _rate=1.0, hz=100.0):
        if (_startTimestamp==0):
            self.startTimestamp = time.time()
        else:
            self.startTimestamp = _startTimestamp
        self.rate = _rate
        self.timeIncrement = (1.0 / float(hz))
        self.delay = self.timeIncrement / _rate
        self._isPause = mp.Event()
        self._isPause.clear()
        self._isStop = mp.Event()
        self._isStop.clear()
        self.clockPub = rospy.Publisher ('/clock', Clock, queue_size=1)
        
        # Last
        self.proc = mp.Process(target=self._process)
        self.proc.start()
        
    def pause (self):
        self._isPause.set()
        
    def resume (self):
        self._isPause.clear()
    
    def close (self):
        self._isPause.set()
        self._isStop.set()
        self.proc.join()
    
    def _process (self):
        i = 0
        curTime = self.startTimestamp
        print ("Init time 0")
        
        while (True):
            if (self._isStop.isSet()):
                break
            
            # Publish current clock
            ck = Clock()
            ck.clock = rospy.Time.from_sec(curTime)
            self.clockPub.publish(ck)
            
            time.sleep(self.delay)
            
            if (not self._isPause.isSet()):
                curTime += self.timeIncrement
            