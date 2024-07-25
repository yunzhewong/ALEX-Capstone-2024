from typing import List
import aiosv2.aios as aios
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
import time
import math


def func(exoMotor: RightKneeExoMotor, runningTime: float):
    exoMotor.motor.setVelocity(math.pi / 8) 



if __name__ == "__main__":
    setup_teardown_rightknee_exomotor(func, 17)
    