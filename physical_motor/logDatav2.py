import math
import time
import sys
import os

# Ensure the parent directory is in the PYTHONPATH
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from classes.DataLog import DataLog
from dataGathering import gather_data
from aiosv2.SafeMotorOperation import AiosSocket, SafeMotor, SafetyConfiguration

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

def calculate_position(running_time: float):
    return sum(coef * (running_time ** i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS))

if __name__ == "__main__":
    try:
        socket = AiosSocket()
        config = SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=4*math.pi, minimum_position=-2 * math.pi / 3, maximum_position=2 * math.pi / 3)
        topMotor = SafeMotor("10.10.10.16", socket, config)
        bottomMotor = SafeMotor("10.10.10.17", socket, config)

        topMotor.enable()
        bottomMotor.enable()

        time.sleep(1)

        command_func = lambda top_motor, bottom_motor, running_time: (
            top_motor.setPosition(calculate_position(running_time)),
            bottom_motor.setPosition(calculate_position(running_time))
        )

        gather_data(command_func, DURATION, SAVE_NAME)

    finally:
        topMotor.disable()
        bottomMotor.disable()
