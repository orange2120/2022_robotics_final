import time 
from time import sleep

class PID:
    def __init__(self, kP, kI, kD):
        #initialize gains
        self.kP = kP
        self.kI = kI
        self.kD = kD
        
    def initialize(self):
        #initialise the current and previous time
        self.currTime = time.time()
        self.prevTime = self.currTime
        
        #initialise the previous error
        self.prevError = 0
        
        #initialise the term result variables
        self.cP = 0
        self.cI = 0
        self.cD = 0
    
    def update(self, error, sleep=0):
        #pause for a bit
        time.sleep(sleep)
        
        #grab the current time and calculate dt
        self.currTime = time.time()
        dt = self.currTime - self.prevTime
        
        #dErr
        dErr = error - self.prevError
        
        #Calculate PID terms
        self.cP = error
        self.cI += error*dt
        self.cD = (dErr/dt) if dt > 0 else 0
        
        #save previous time and error for the next update
        self.prevTime = self.currTime
        self.prevError = error
        
        #Sum the terms and return
        return sum([self.kP*self.cP, self.kI*self.cI, self.kD*self.cD]) 