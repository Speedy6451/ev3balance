import time
class PID:
    def __init__(self,kp,ki,kd,k1=1,k2=0):
        self.kp = kp # proportional gain
        self.ki = ki # integral gain
        self.kd = kd # derivitave gain
        self.i = 0   # integral
        self.le = 0  # last error
        self.k1 = k1 # 1st input gain
        self.k2 = k2 # 2nd input gain
        self.lt = time.time() # last time
    def loop(self,e1,e2=0,d=False):
        e = self.k1 * e1 + self.k2 * e2 
        t = time.time()
        dt = t - self.lt # delta time
        self.i = self.i + e # integral
        d = (e - self.le)/dt
        o = e * self.kp + self.i * self.ki + d + self.kd # output
        self.le = e # the past is now
        self.lt = t
        if d: # debug
            print("""
            dt: {}
            i: {}
            p: {}
            d: {}
            o: {}
            """.format(dt,self.i,e,d,o))
        return o
        
