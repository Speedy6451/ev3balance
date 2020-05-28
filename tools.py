#!/usr/bin/env python3 from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_3
from ev3dev2.sensor.lego import TouchSensor, GyroSensor
from ev3dev2.led import Leds
import time

class Robot:
    def __init__(self):
        # Motors
        self.m1 = LargeMotor(OUTPUT_B)
        self.m2 = LargeMotor(OUTPUT_C)

        # Sensors
        self.g = GyroSensor(INPUT_3)
    def drive(self, go, turn): # drive with the specified forward and steer values
        self.m1.on(SpeedPercent(clampa(go + turn, 100)))
        self.m2.on(SpeedPercent(clampa(go - turn, 100)))
    def stop(self): # self documenting
        self.m1.stop()
        self.m2.stop()

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
        self.i = self.i + (e * dt) # integral
        d = (e - self.le)/dt
        o = (e * self.kp) + (self.i * self.ki) + (d * self.kd) # output
        if (not(e > 0 and self.le > 0)):
            self.i = 0
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
        
def clamp(num, minn, maxn): # constrains a number between two others
    return max(min(maxn, num), minn)

def clampa(num, maxn): # clamp, but one number
    return max(min(maxn, num), 0-maxn)
