from enum import Enum
from typing import List
import aiosv2.CSVWriter
from aiosv2.SafeMotorOperation import SafeMotor
import aiosv2.aios as aios
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
import time
import math
from aiosv2.constants import ControlMode
import matplotlib.pyplot as plt
import numpy as np

        
class State():
    def __init__(self):
        pass

    
if __name__ == "__main__":

    state = State()
    def func(exoMotor: RightKneeExoMotor, runningTime: float):
        exoMotor.motor.setCurrent(2)
        print(exoMotor.motor.getCVP())

    setup_teardown_rightknee_exomotor(func, 20)
    