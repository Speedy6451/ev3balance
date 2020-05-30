#!/usr/bin/env python3
from tools import Balance, PID

b = Balance(PID(2.2,0,1,1,.2),54) 
b.r.resetGyro()
try:
    while True:
        b.r.balanceLoop(0,0)
finally:
    b.r.stop()
