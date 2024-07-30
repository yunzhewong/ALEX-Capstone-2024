from enum import Enum
from typing import List
import aiosv2.aios as aios
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
import time
import math
from aiosv2.constants import ControlMode

def func(exoMotor: RightKneeExoMotor, runningTime: float):
    exoMotor.motor.modeChangeIfNecessary(ControlMode.Position)
    aios.getEncoderInfo(exoMotor.motor.getIP())
    aios.setInputPosition_pt(exoMotor.motor.getIP(), 9, 0, 0)
    print(exoMotor.motor.getCVP())
    
if __name__ == "__main__":
    setup_teardown_rightknee_exomotor(func, 10)
    