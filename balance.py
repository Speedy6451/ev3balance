#!/usr/bin/env python3
import time
from tools import Robot, PID
import tools

# Robot
r = Robot()

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
    target = r.g.value() - balance
    lastTime = time.time()

def isUpright(angle): # Calculates if the robot will be able to right itself,
    global upright
    if (angle > balance - tipMin and angle < balance + tipMax):
        upright = True
        return True
    upright = False
    return False

def balanceLoop(angle, rotate): # call every loop to balance
    gyro = r.g.value()
    error = (target - gyro) + angle
    out = p.loop(error)
    r.drive(out, rotate)

def hold(): # Balances upright until end of time
    resetGyro()
    while True:
        balanceLoop(0,0)

if __name__ == '__main__':
    try:
        hold()
    finally:
        r.stop()
