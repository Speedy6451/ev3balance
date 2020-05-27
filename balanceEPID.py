#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_3
from ev3dev2.sensor.lego import TouchSensor, GyroSensor
from ev3dev2.led import Leds
import time
from pid import PID

# Motors
m1 = LargeMotor(OUTPUT_B)
m2 = LargeMotor(OUTPUT_C)

# Sensors
g = GyroSensor(INPUT_3)

# PIDs
p = PID(1.8,.6,.20)

# Constants
balance = 46 # gyro ange of balance
tipMax = 25 # max above target before quit
tipMin = 45 # max below target before quit

# Vars
target = 0
error = 0
upright = False
lastTime = time.time()

# Functions
def resetGyro(): # resets the gyro. use when tobot is lying down on a flat surface.
    global target, lastTime
    target = g.value() - balance
    lastTime = time.time()

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
    return max(min(maxn, num), 0-maxn)

def drive(go, turn): # drive with the specified forward and steer values
    m1.on(SpeedPercent(clampa(go + turn, 100)))
    m2.on(SpeedPercent(clampa(go - turn, 100)))

def balanceLoop(angle, rotate): # call every loop to balance
    gyro = g.value()
    error = (target - gyro) + angle
    out = p.loop(error)
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
