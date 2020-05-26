#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_3
from ev3dev2.sensor.lego import TouchSensor, GyroSensor
from ev3dev2.led import Leds
import time

# Motors
m1 = LargeMotor(OUTPUT_B)
m2 = LargeMotor(OUTPUT_C)

# Sensors
g = GyroSensor(INPUT_3)

# Constants
balance = 55 # gyro ange of balance
p = 1 # proportional gain
i = .25 # integral gain
d = .1 # derivitave gain

tipMax = 25 # max above target before quit
tipMin = 45 # max below target before quit

# Vars
target = 0
error = 0
lastError = 0
upright = False
out = 0
integral = 0
derivative = 0
lastTime = time.time()

# Functions
def resetGyro(): # resets the gyro. use when tobot is lying down on a flat surface.
    global target, lastTime
    target = g.value() + balance
    lastTime = time.time()

def pid(error): # calculates speed from error
    global integral, derivative, lastError, lastTime
    currentTime = time.time()
    delta = (currentTime - lastTime)
    integral += error * delta
    lastTime = currentTime
    derivative = error - lastError
    out = error * p + integral * i + derivative  * d
    lastError = error
    print("""
    delta: {}
    i: {}
    p: {}
    d: {}
    o: {}
    """.format(delta,integral,error,derivative,out))
    return out

def isUpright(angle): # Calculates if the robot will be able to right itself,
    global upright
    if (angle > balance - tipMin and angle < balance + tipMax):
        upright = True
        return True
    upright = False
    return False

def stop(): # self documenting
    m1.stop()
    m2.stop()

def clamp(num, minn, maxn): # constrains a number between two others
    return max(min(maxn, num), minn)

def clampa(num, maxn): # clamp, but one number
    return max(min(maxn, num), maxn)

def drive(go, turn): # drive with the specified forward and steer values
    #m1.on(SpeedPercent(clampa(go + turn, 100)))
    #m2.on(SpeedPercent(clampa(go - turn, 100)))
    print("not runnng")

def balanceLoop(angle, rotate): # call every loop to balance
    gyro = g.value()
    error = (target - gyro) + angle
    out = pid(error)
    drive(out, rotate)

def hold(): # Balances upright until end of time
    resetGyro()
    while True:
        balanceLoop(0,0)

if __name__ == '__main__':
    try:
        hold()
    finally:
        stop()
