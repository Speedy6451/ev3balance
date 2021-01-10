#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_3
from ev3dev2.sensor.lego import TouchSensor, GyroSensor
from ev3dev2.led import Leds
import time

class Robot: # Hardware class
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

class PID: # PID class
    def __init__(self,kp,ki,kd,k1=1,k2=0):
        self.kp = kp # proportional gain
        self.ki = ki # integral gain
        self.kd = kd # derivitave gain
        self.i = 0   # integral
        self.le = 0  # last error
        self.k1 = k1 # 1st input gain
        self.k2 = k2 # 2nd input gain
        self.t = Dt() # delta time

    def loop(self,e1,e2=0,d=False):
        e = self.k1 * e1 + self.k2 * e2 
        self.t.update() # update delta time
        self.i = self.i + (e * self.t.dt) # integral
        d = (e - self.le)/self.t.dt
        o = (e * self.kp) + (self.i * self.ki) + (d * self.kd) # output
        if (not(e > 0 and self.le > 0)):
            self.i = 0
        self.le = e # the past is now
        if d: # debug
            print("""
            dt: {}
            i: {}
            p: {}
            d: {}
            o: {}
            """.format(self.t.dt,self.i,e,d,o))
        return o

class Dt: # Delta time class
    def __init__(self):
        self.time = time.time()
        self.lastTime = self.time
        self.dt = 0

    def update(self): # Find time since last update
        self.time = time.time()
        self.dt = self.lastTime - self.time
        self.lastTime = self.time
        return self.dt

class Balance:
    def __init__(self, pid=PID(1.8,.6,.20), balance=46, robot=Robot()):
        self.pid = pid
        self.r = robot
        self.balance = balance
        self.tipMax = 25
        self.tipMin = 45
        self.target = 0 
        self.error = 0
        self.upright = False
        self.resetGyro()

    def resetGyro(self):
        self.target = self.r.g.value() - self.balance

    def isUpright(self, angle):
        if (angle > self.balance - self.tipMin and angle < self.balance + self.tipMax):
            self.upright = True
            return True
        self.upright = False
        return False

    def balanceLoop(self, angle, rotate): # call every loop to balance
        gyro = self.r.g.value()
        error = (self.target - gyro) + angle
        out = self.pid.loop(error)
        self.r.drive(out, rotate)

    def balanceMotors(self, angle, rotate): # call every loop to balance
        gyro = self.r.g.value()
        m = (self.r.m1.position + self.r.m2.position) / 2
        error = (self.target - gyro) + angle
        error2 = 0 - m
        out = self.pid.loop(error, error2)
        self.r.drive(out, rotate)

    def hold(self): # Balances upright until end of time
        self.resetGyro()
        try:
            while True:
                self.balanceLoop(0,0)
        finally:
            self.r.stop()

def clamp(num, minn, maxn): # constrains a number between two others
    return max(min(maxn, num), minn)

def clampa(num, maxn): # clamp, but one number
    return max(min(maxn, num), 0-maxn)
