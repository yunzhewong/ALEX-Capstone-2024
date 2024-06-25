import math
import time
import aiosv2
import numpy as np
import matplotlib.pyplot as plt
from classes.DataLog import DataLog
from dataGathering import gather_data

SAVE_NAME = "polynomial_position_motor.csv"
DURATION = 5

SAMPLE_PERIOD = 0.04
POLYNOMIAL_COEFFICIENTS = [0.01, 0.1, 0.01, 0.01]

def get_poly_time(runningTime: float):
    CYCLE_DURATION = 4.0 
    cycle_time = runningTime % CYCLE_DURATION

    if cycle_time > CYCLE_DURATION / 2:
        return CYCLE_DURATION - cycle_time
    return cycle_time


if __name__ == "__main__":
    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        poly_time = get_poly_time(runningTime) 
        position = np.polyval(POLYNOMIAL_COEFFICIENTS, poly_time)
        connection.setPosition(position)


    time.sleep(1)
    gather_data(command_func, DURATION, SAVE_NAME)

