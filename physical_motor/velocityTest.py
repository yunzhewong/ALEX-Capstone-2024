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
DURATION = 10


def cal_velocity(p, running_time):
    return p * running_time


# def return_to_start_position():

if __name__ == "__main__":

    def command_func(top_motor: SafeMotor, bottom_motor: SafeMotor, running_time):

        top_motor.setVelocity(0.1)

    gather_data(command_func, DURATION, SAVE_NAME)
