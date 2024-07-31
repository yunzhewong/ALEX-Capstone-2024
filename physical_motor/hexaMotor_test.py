import math
import time
import sys
import os

# Ensure the parent directory is in the PYTHONPATH
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# from classes.DataLog_leftKnee import DataLog_leftKnee
from gatherData_Hexa import gather_data
from aiosv2.SafeMotorOperation import SafeMotor

SAVE_NAME = "hexa motor test.csv"
DURATION = 5

if __name__ == "__main__":

    def command_func(kneeLeft: SafeMotor, running_time):

        kneeLeft.setVelocity(0)

    gather_data(command_func, DURATION, SAVE_NAME)
