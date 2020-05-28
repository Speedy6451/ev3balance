#!/usr/bin/env python3
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
    def clamp(self, num, minn, maxn): # constrains a number between two others
        return max(min(maxn, num), minn)

    def clampa(self, num, maxn): # clamp, but one number
        return max(min(maxn, num), 0-maxn)
    def drive(self, go, turn): # drive with the specified forward and steer values
        self.m1.on(SpeedPercent(self.clampa(go + turn, 100)))
        self.m2.on(SpeedPercent(self.clampa(go - turn, 100)))
    def stop(self): # self documenting
        self.m1.stop()
        self.m2.stop()
