import sys
import os

# Ensure the parent directory is in the PYTHONPATH
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dataGathering import gather_data
from aiosv2.SafeMotorOperation import SafeMotor

SAVE_NAME = "polynomial_position_motor.csv"
DURATION = 7


def reset_position(motor: SafeMotor):

    if motor.getCVP().position < 0:
        motor.setVelocity(0.5)
    else:
        motor.setVelocity(0)


if __name__ == "__main__":

    def command_func(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):

        bottom_motor.setPosition(0)

        if running_time <= 5:
            top_motor.setCurrent(-5)

        if running_time > 5:
            reset_position(top_motor)

    gather_data(command_func, DURATION, SAVE_NAME)
