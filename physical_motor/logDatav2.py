import math
import time
import aiosv2
import numpy as np
import matplotlib.pyplot as plt
from classes.DataLog import DataLog
from dataGathering import gather_data
from aiosv2 import AiosSocket, SafeMotor, SafetyConfiguration

SAVE_NAME = "polynomial_position_motor.csv"
DURATION = 5

SAMPLE_PERIOD = 0.04
POLYNOMIAL_COEFFICIENTS = [0.016, 0.04, 0.0, 0.0]

def get_poly_time(runningTime: float):
    CYCLE_DURATION = 5.0 
    cycle_time = runningTime % CYCLE_DURATION

    if cycle_time > CYCLE_DURATION / 2:
        return CYCLE_DURATION - cycle_time
    return cycle_time


if __name__ == "__main__":
    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        poly_time = get_poly_time(runningTime) 
        position = np.polyval(POLYNOMIAL_COEFFICIENTS, poly_time)
        twin_motor.topMotor.setPosition(position)
        twin_motor.bottomMotor.setPosition(position)

    try:
        socket = AiosSocket()
        config = SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=4*math.pi, minimum_position=-2 * math.pi / 3, maximum_position=2 * math.pi / 3)
        topMotor = SafeMotor("10.10.10.16", socket, config)
        bottomMotor = SafeMotor("10.10.10.17", socket, config)

        topMotor.enable()
        bottomMotor.enable()

        time.sleep(1)
        gather_data(lambda _, t: command_func(topMotor, bottomMotor, t), DURATION, SAVE_NAME)

    finally:
        topMotor.disable()
        bottomMotor.disable()