from enum import Enum
from typing import List
import aiosv2.aios as aios
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
import time
import math
from aiosv2.constants import ControlMode


class InputMode(Enum):
    Current = 1
    Velocity = 2
    Position = 3

def func(exoMotor: RightKneeExoMotor, runningTime: float):
    aios.inputMode(2, exoMotor.motor.getIP(), 1)
    exoMotor.motor.setVelocity(1)
    print(exoMotor.motor.getCVP())
    



if __name__ == "__main__":
    setup_teardown_rightknee_exomotor(func, 10)
    