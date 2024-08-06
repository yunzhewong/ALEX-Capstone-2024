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
    (3 * 2*math.pi / 200),
    (-1 * 2*math.pi / 1000),
]  # Coefficients for the top position reference
POLY_COEFFICIENTS_BOTTOM = [
    0.0,
    2*(3 * math.pi / 200),
    3*(-1 * math.pi / 1000),
]  # Coefficients for the bottom position reference
POLY_COEFFICIENTS_TOP = [
    0.0,
    2*(3 * 2*math.pi / 200), 
    3*(-1 * 2*math.pi / 1000),
]  # Coefficients for the top position reference

DESIRED_TRAJECTORY_BOTTOM = [0.0, 0.0, (3 * math.pi / 200), (-1 * math.pi / 1000)]
DESIRED_TRAJECTORY_TOP = [0.0, 0.0, (3 * 2 * math.pi / 200), (-1 * 2 * math.pi / 1000)]

DESIRED_VELOCITY_BOTTOM = [0.0, 2*(3 * math.pi / 200), 3*(-1 * math.pi / 1000)]
DESIRED_VELOCITY_TOP = [0.0, 2*(3 * 2 * math.pi / 200), 3*(-1 * 2 * math.pi / 1000)]


# PID controller parameters
Kp = 1.0
Ki = 0.1
Kd = 0.05

# PID controller state
integral_error_top = 0.0
previous_error_top = 0.0
integral_error_bottom = 0.0
previous_error_bottom = 0.0


def calculate_refPosition_Bottom(running_time: float):
    return sum(
        coef * (running_time**i)
        for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_BOTTOM)
    )


def calculate_refPosition_Top(running_time: float):
    return sum(
        coef * (running_time**i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_TOP)
    )


def calculate_refVelocity_Bottom(running_time: float):
    return sum(
        coef * (running_time**i) for i, coef in enumerate(POLY_COEFFICIENTS_BOTTOM)
    )


def calculate_refVelocity_Top(running_time: float):
    return sum(coef * (running_time**i) for i, coef in enumerate(POLY_COEFFICIENTS_TOP))


def pid_controller(error, integral_error, previous_error, dt):
    P = Kp * error
    I = Ki * integral_error
    D = Kd * (error - previous_error) / dt
    return P + I + D



if __name__ == "__main__":

    def command_func(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
        global integral_error_top, previous_error_top, integral_error_bottom, previous_error_bottom

        # Calculate reference position bottom
        ref_position_bottom = calculate_refPosition_Bottom(running_time)

        # Calculate reference position top
        ref_position_top = calculate_refPosition_Top(running_time)

        # Get current positions
        current_position_top = (
            top_motor.getCVP().position if top_motor.getCVP() is not None else 0.0
        )
        current_position_bottom = (
            bottom_motor.getCVP().position if bottom_motor.getCVP() is not None else 0.0
        )

        # Calculate errors
        error_top = ref_position_top - current_position_top
        error_bottom = ref_position_bottom - current_position_bottom

        # Update integral and derivative errors
        integral_error_top += error_top * SAMPLE_PERIOD
        integral_error_bottom += error_bottom * SAMPLE_PERIOD

        # Calculate control outputs
        velocity_top = pid_controller(
            error_top, integral_error_top, previous_error_top, SAMPLE_PERIOD
        ) + calculate_refVelocity_Top(running_time)

        velocity_bottom = pid_controller(
            error_bottom, integral_error_bottom, previous_error_bottom, SAMPLE_PERIOD
        ) + calculate_refVelocity_Bottom(running_time)

        # Incorporate desired trajectory and velocity
        desired_position_bottom = sum(
            coef * (running_time**i) for i, coef in enumerate(DESIRED_TRAJECTORY_BOTTOM)
        )
        desired_position_top = sum(
            coef * (running_time**i) for i, coef in enumerate(DESIRED_TRAJECTORY_TOP)
        )

        desired_velocity_bottom = sum(
            coef * (running_time**i) for i, coef in enumerate(DESIRED_VELOCITY_BOTTOM)
        )
        desired_velocity_top = sum(
            coef * (running_time**i) for i, coef in enumerate(DESIRED_VELOCITY_TOP)
        )

        # Adjust control outputs based on desired values
        velocity_top += desired_velocity_top - calculate_refVelocity_Top(running_time)
        velocity_bottom += desired_velocity_bottom - calculate_refVelocity_Bottom(running_time)

        # Update motors' velocity
        top_motor.setVelocity(velocity_top)
        bottom_motor.setVelocity(velocity_bottom)

        # Update previous errors
        previous_error_top = error_top
        previous_error_bottom = error_bottom

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

