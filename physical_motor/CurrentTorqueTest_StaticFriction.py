import math
import time
import aiosv2
import numpy as np
import matplotlib.pyplot as plt
from classes.DataLog import DataLog
from typing import Callable
import aiosv2
from classes.DataLog import DataLog
from aiosv2.TwinMotor import setup_teardown_twin_motor

SAVE_NAME = "polynomial_position_motor.csv"
DURATION = 10
POLYNOMIAL_COEFFICIENTS = [-0.2, -1]
flag = 0

class MotorStartMoving(Exception):
    pass

def get_time(runningTime: float):
    CYCLE_DURATION = 10.0 
    cycle_time = runningTime % CYCLE_DURATION

    if cycle_time > CYCLE_DURATION / 2:
        return CYCLE_DURATION - cycle_time
    return cycle_time

def gather_data(command_func: Callable[[aiosv2.SafeMotor, float], None], duration: float, name: str):
    dataLog = DataLog()

    def func(twinMotor: aiosv2.TwinMotor, runningTime: float):
        connection = twinMotor.topMotor
        command_func(connection, runningTime)
        cvp = connection.getCVP()
            

        dataLog.addCVP(runningTime, cvp)

    setup_teardown_twin_motor(func, duration)

    dataLog.plot()
    dataLog.download(name)


if __name__ == "__main__":
    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        # time = get_time(runningTime)
        current = np.polyval(POLYNOMIAL_COEFFICIENTS, runningTime)
        connection.setCurrent(current)

    time.sleep(1)
    gather_data(command_func, DURATION, SAVE_NAME)

