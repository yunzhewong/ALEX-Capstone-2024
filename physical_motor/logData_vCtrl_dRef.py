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
DURATION = 10

SAMPLE_PERIOD = 0.04
POLYNOMIAL_COEFFICIENTS_BOTTOM = [0.0, 0.04, 0.016]  # Coefficients for the bottom position reference
POLYNOMIAL_COEFFICIENTS_TOP = [0.0, 0.1, 0.016]  # Coefficients for the top position reference
POLY_COEFFICIENTS_BOTTOM = [0.08, 0.048]  # Coefficients for the bottom position reference
POLY_COEFFICIENTS_TOP = [0.2, 0.048]  # Coefficients for the top position reference

# POLYNOMIAL_COEFFICIENTS_TOPV = [0.0, 0.2, 0.048]  # Coefficients for the top motor velocity
# POLYNOMIAL_COEFFICIENTS_BOTTOMV = [0.0, 0.08, 0.048]  # Coefficients for the bottom motor velocity

# PID controller parameters
Kp = 1.0
Ki = 0.1
Kd = 0.05

# PID controller state
integral_error_top = 0.0
previous_error_top = 0.0
integral_error_bottom = 0.0
previous_error_bottom = 0.0

# def get_poly_time(runningTime: float):
#     CYCLE_DURATION = 5.0 
#     cycle_time = runningTime % CYCLE_DURATION

#     if cycle_time > CYCLE_DURATION / 2:
#         return CYCLE_DURATION - cycle_time
#     return cycle_time

# def calculate_velocity_top(running_time: float):
#     return sum(coef * (running_time ** i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_TOP))

# def calculate_velocity_bottom(running_time: float):
#     return sum(coef * (running_time ** i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_BOTTOM))

# def reset_velocity(running_time: float, start_position):
#     return max(0, start_position - running_time * (start_position / 5))

# def calculate_position_top(running_time: float):
#     return 0.016 * (running_time ** 3) + 0.1 * (running_time ** 2)

# def calculate_position_bottom(running_time: float):
#     return 0.016 * (running_time ** 3) + 0.04 * (running_time ** 2)

# def reset_position(running_time: float, start_position):
#     return max(0, start_position - running_time * (start_position / 5))

def calculate_refPosition_Bottom(running_time: float):
    return sum(coef * (running_time ** i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_BOTTOM))
def calculate_refPosition_Top(running_time: float):
    return sum(coef * (running_time ** i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS_TOP))

def calculate_refVelocity_Bottom(running_time: float):
    return sum(coef * (running_time ** i) for i, coef in enumerate(POLY_COEFFICIENTS_BOTTOM))
def calculate_refVelocity_Top(running_time: float):
    return sum(coef * (running_time ** i) for i, coef in enumerate(POLY_COEFFICIENTS_TOP))


def pid_controller(error, integral_error, previous_error, dt):
    P = Kp * error
    I = Ki * integral_error
    D = Kd * (error - previous_error) / dt
    return P + I + D

if __name__ == "__main__":

    # def command_func(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
    #     quad_duration = DURATION / 4
    #     if running_time > quad_duration and running_time <= quad_duration * 2:
    #         # Reverse the position for the bottom motor after half the duration
    #         top_motor.setVelocity(calculate_velocity_top(DURATION / 2 - running_time))
    #         bottom_motor.setVelocity(calculate_velocity_bottom(DURATION / 2 - running_time))
    #     elif running_time > quad_duration * 2 and running_time <= quad_duration * 3:
    #         top_motor.setVelocity(-calculate_velocity_top(running_time - quad_duration * 2))
    #         bottom_motor.setVelocity(-calculate_velocity_bottom(running_time - quad_duration * 2))
    #     elif running_time > quad_duration * 3:
    #         top_motor.setVelocity(-calculate_velocity_top(DURATION - running_time))
    #         bottom_motor.setVelocity(-calculate_velocity_bottom(DURATION - running_time))
    #     else:
    #         top_motor.setVelocity(calculate_velocity_top(running_time))
    #         bottom_motor.setVelocity(calculate_velocity_bottom(running_time))

    # # Reset the motor's position to zero
    # def command_func1(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
    #     if top_motor.getCVP() is None or bottom_motor.getCVP() is None:
    #         return
    #     x1 = top_motor.getCVP().position
    #     x2 = bottom_motor.getCVP().position
    #     top_motor.setPosition(reset_position(running_time, x1))
    #     bottom_motor.setPosition(reset_position(running_time, x2))
    def command_func(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
        global integral_error_top, previous_error_top, integral_error_bottom, previous_error_bottom

        # Calculate reference position bottom
        ref_position_bottom = calculate_refPosition_Bottom(running_time)

        # Calculate reference position top
        ref_position_top = calculate_refPosition_Top(running_time)
        
        # Get current positions
        current_position_top = top_motor.getCVP().position if top_motor.getCVP() is not None else 0.0
        current_position_bottom = bottom_motor.getCVP().position if bottom_motor.getCVP() is not None else 0.0

        # Calculate errors
        error_top = ref_position_top - current_position_top
        error_bottom = ref_position_bottom - current_position_bottom

        # Update integral and derivative errors
        integral_error_top += error_top * SAMPLE_PERIOD
        integral_error_bottom += error_bottom * SAMPLE_PERIOD

        # Calculate control outputs
        velocity_top = pid_controller(error_top, integral_error_top, previous_error_top, SAMPLE_PERIOD) + calculate_refVelocity_Top(running_time)
        velocity_bottom = pid_controller(error_bottom, integral_error_bottom, previous_error_bottom, SAMPLE_PERIOD) + calculate_refVelocity_Bottom(running_time)

        # Update motors' velocity
        top_motor.setVelocity(velocity_top)
        bottom_motor.setVelocity(velocity_bottom)

        # Update previous errors
        previous_error_top = error_top
        previous_error_bottom = error_bottom

    gather_data(command_func, DURATION, SAVE_NAME)