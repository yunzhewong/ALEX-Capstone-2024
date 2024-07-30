import math
import time
import sys
import os

# Ensure the parent directory is in the PYTHONPATH
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from classes.DataLog import DataLog
from dataGathering import gather_data
from aiosv2.SafeMotorOperation import SafeMotor

SAVE_NAME = "polynomial_position_motor.csv"
DURATION = 5

SAMPLE_PERIOD = 0.04
POLYNOMIAL_COEFFICIENTS_TOP = [0.0, 0.1, 0.016]  # Coefficients for the top motor
POLYNOMIAL_COEFFICIENTS_BOTTOM = [0.0, 0.04, 0.016]  # Coefficients for the bottom motor


def get_poly_time(runningTime: float):
    CYCLE_DURATION = 5.0
    cycle_time = runningTime % CYCLE_DURATION

    if cycle_time > CYCLE_DURATION / 2:
        return CYCLE_DURATION - cycle_time
    return cycle_time


def calculate_position_top(running_time: float):
    return sum(
        coef * (running_time**i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_TOP)
    )


def calculate_position_bottom(running_time: float):
    return sum(
        coef * (running_time**i)
        for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_BOTTOM)
    )


def reset_position(running_time: float, start_position):
    return max(0, start_position - running_time * (start_position / 5))


if __name__ == "__main__":

    def command_func1(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
        half_duration = DURATION / 2
        if running_time > half_duration:
            # Reverse the position for the bottom motor after half the duration
            top_motor.setPosition(calculate_position_top(DURATION - running_time))
            bottom_motor.setPosition(calculate_position_bottom(DURATION - running_time))
        else:
            top_motor.setPosition(calculate_position_top(running_time))
            bottom_motor.setPosition(calculate_position_bottom(running_time))

    # Reset the motor's position to zero
    def command_func(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
        if top_motor.getCVP() is None or bottom_motor.getCVP() is None:
            return
        x1 = top_motor.getCVP().position
        x2 = bottom_motor.getCVP().position
        top_motor.setPosition(reset_position(running_time, x1))
        bottom_motor.setPosition(reset_position(running_time, x2))

    gather_data(command_func, DURATION, SAVE_NAME)
