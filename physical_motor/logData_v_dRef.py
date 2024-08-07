import math
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# Ensure the parent directory is in the PYTHONPATH
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from classes.DataLog import DataLog
from dataGathering import gather_data
from aiosv2.SafeMotorOperation import SafeMotor

SAVE_NAME = "polynomial_position_motor.csv"
DURATION = 10

SAMPLE_PERIOD = 0.04
POLYNOMIAL_COEFFICIENTS_BOTTOM = [
    0.0,
    0.0,
    (3 * math.pi / 200),
    (-1 * math.pi / 1000),
]  # Coefficients for the bottom position reference
POLYNOMIAL_COEFFICIENTS_TOP = [
    0.0,
    0.0,
    (3 * 2 * math.pi / 200),
    (-1 * 2 * math.pi / 1000),
]  # Coefficients for the top position reference
POLY_COEFFICIENTS_BOTTOM = [
    0.0,
    2 * (3 * math.pi / 200),
    3 * (-1 * math.pi / 1000),
]  # Coefficients for the bottom position reference
POLY_COEFFICIENTS_TOP = [
    0.0,
    2 * (3 * 2 * math.pi / 200),
    3 * (-1 * 2 * math.pi / 1000),
]  # Coefficients for the top position reference

DESIRED_TRAJECTORY_BOTTOM = [0.0, 0.0, (3 * math.pi / 200), (-1 * math.pi / 1000)]
DESIRED_TRAJECTORY_TOP = [0.0, 0.0, (3 * 2 * math.pi / 200), (-1 * 2 * math.pi / 1000)]

DESIRED_VELOCITY_BOTTOM = [0.0, 2 * (3 * math.pi / 200), 3 * (-1 * math.pi / 1000)]
DESIRED_VELOCITY_TOP = [0.0, 2 * (3 * 2 * math.pi / 200), 3 * (-1 * 2 * math.pi / 1000)]


def calculate_refPosition_Bottom(running_time: float):
    return sum(
        coef * (running_time ** i)
        for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_BOTTOM)
    )


def calculate_refPosition_Top(running_time: float):
    return sum(
        coef * (running_time ** i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_TOP)
    )


def calculate_refVelocity_Bottom(running_time: float):
    return sum(
        coef * (running_time ** i) for i, coef in enumerate(POLY_COEFFICIENTS_BOTTOM)
    )


def calculate_refVelocity_Top(running_time: float):
    return sum(coef * (running_time ** i) for i, coef in enumerate(POLY_COEFFICIENTS_TOP))


if __name__ == "__main__":

    def command_func(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
        # Calculate reference velocities
        ref_velocity_bottom = calculate_refVelocity_Bottom(running_time)
        ref_velocity_top = calculate_refVelocity_Top(running_time)

        # Set motors' velocity
        top_motor.setVelocity(ref_velocity_top)
        bottom_motor.setVelocity(ref_velocity_bottom)

        # Plotting and saving data
        dataLog = DataLog(
            desired_trajectory_bottom=DESIRED_TRAJECTORY_BOTTOM,
            desired_trajectory_top=DESIRED_TRAJECTORY_TOP,
            desired_velocity_bottom=DESIRED_VELOCITY_BOTTOM,
            desired_velocity_top=DESIRED_VELOCITY_TOP,
        )

    gather_data(
        command_func,
        DURATION,
        SAVE_NAME,
        desired_trajectory_bottom=DESIRED_TRAJECTORY_BOTTOM,
        desired_trajectory_top=DESIRED_TRAJECTORY_TOP,
        desired_velocity_bottom=DESIRED_VELOCITY_BOTTOM,
        desired_velocity_top=DESIRED_VELOCITY_TOP
    )
