#!/usr/bin/env python3
from tools import Balance, PID

b = Balance(PID(2.2,0,1,1,.08),54) 
b.resetGyro()
try:
    while True:
        b.balanceLoop(0,0)
finally:
    b.r.stop()
