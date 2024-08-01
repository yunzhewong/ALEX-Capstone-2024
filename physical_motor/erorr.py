from typing import List
import aiosv2
from aiosv2.TwinMotor import setup_teardown_twin_motor
import time
import math

def func(twinMotor: aiosv2.TwinMotor, runningTime: float):
    bottomMotor = twinMotor.bottomMotor
    bottomMotor.setPosition(math.pi / 4)

if __name__ == "__main__":
    setup_teardown_twin_motor(func, 5)
    