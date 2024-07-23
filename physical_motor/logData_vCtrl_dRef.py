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
POLYNOMIAL_COEFFICIENTS = [0.0, 0.08, 0.048]

def get_poly_time(runningTime: float):
    CYCLE_DURATION = 5.0 
    cycle_time = runningTime % CYCLE_DURATION

    if cycle_time > CYCLE_DURATION / 2:
        return CYCLE_DURATION - cycle_time
    return cycle_time

def calculate_velocity(running_time: float):
    return sum(coef * (running_time ** i) for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS))

def reset_velocity(running_time: float, start_position):
    return max(0, start_position - running_time * (start_position / 5))

def calculate_position(running_time: float):
    return 0.016 * (running_time ** 3) + 0.04 * (running_time ** 2)

def reset_position(running_time: float, start_position):
    return max(0, start_position - running_time * (start_position / 5))

if __name__ == "__main__":
    try:
        socket = AiosSocket()
        config = SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=4 * math.pi, minimum_position=-2 * math.pi / 3, maximum_position=2 * math.pi / 3)
        topMotor = SafeMotor("10.10.10.16", socket, config)
        bottomMotor = SafeMotor("10.10.10.17", socket, config)

        topMotor.enable()
        bottomMotor.enable()

        time.sleep(1)

        def command_func(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
            quad_duration = DURATION / 4
            if running_time > quad_duration and running_time <= quad_duration * 2:
                # Reverse the position for the bottom motor after half the duration
                top_motor.setVelocity(calculate_velocity(DURATION / 2 - running_time))
                bottom_motor.setVelocity(calculate_velocity(DURATION / 2 - running_time))
            elif running_time > quad_duration * 2 and running_time <= quad_duration * 3:
                top_motor.setVelocity(-calculate_velocity(running_time - quad_duration * 2))
                bottom_motor.setVelocity(-calculate_velocity(running_time - quad_duration * 2))
            elif running_time > quad_duration * 3:
                top_motor.setVelocity(-calculate_velocity(DURATION - running_time))
                bottom_motor.setVelocity(-calculate_velocity(DURATION - running_time))
            else:
                top_motor.setVelocity(calculate_velocity(running_time))
                bottom_motor.setVelocity(calculate_velocity(running_time))

        # Reset the motor's position to zero
        def command_func1(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):
            if top_motor.getCVP() is None or bottom_motor.getCVP() is None:
                return
            x1 = top_motor.getCVP().position
            x2 = bottom_motor.getCVP().position
            top_motor.setPosition(reset_position(running_time, x1))
            bottom_motor.setPosition(reset_position(running_time, x2))

        gather_data(command_func, DURATION, SAVE_NAME)

    finally:
        topMotor.disable()
        bottomMotor.disable()
