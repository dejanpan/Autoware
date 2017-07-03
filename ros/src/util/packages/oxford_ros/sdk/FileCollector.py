import os
import sys
import multiprocessing.dummy as mp
from collections import deque
from copy import copy


class FileCollector:
    
    def __init__ (self, _fileList, _ReadFunc, queue_length=10):
        self.length = queue_length
        self.queue = deque(maxlen=self.length)
        self.ReadFunc = _ReadFunc
        self.fileList = _fileList
        self.cond = mp.Condition()
        self.process = mp.Process(target=self.producer)
        self.process.start()
        
    def pick (self):
        self.cond.acquire()
        if (len(self.queue)==0) :
            self.cond.wait()
        # XXX
        val = copy(self.queue.popleft())
        self.cond.release()
        return val
    
    def producer (self):
        i = 0
        while True:
            self.cond.acquire()
            if len(self.queue) < self.length:
                curData = self.ReadFunc(self.fileList[i])
                i += 1
                self.queue.append(curData)
            else:
                pass
            self.cond.notifyAll()
            self.cond.release()
#         self.cond.acquire()
#         self.cond.notify()
        