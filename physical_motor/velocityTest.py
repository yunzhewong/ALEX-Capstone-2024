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
DURATION = 7

def cal_velocity(p, running_time):
    return p*running_time







# def return_to_start_position():

if __name__ == "__main__":
    try:
        socket = AiosSocket()
        config = SafetyConfiguration(margin=0.05, maximum_current=2, maximum_velocity=1*math.pi, minimum_position=-1 * math.pi / 3, maximum_position=1 * math.pi / 3)
        topMotor = SafeMotor("10.10.10.16", socket, config)
        bottomMotor = SafeMotor("10.10.10.17", socket, config)

        topMotor.enable()
        bottomMotor.enable()

        time.sleep(1)

        print(topMotor.config.maximum_position)

        def command_func(top_motor: SafeMotor, bottom_motor:SafeMotor, running_time):

            bottom_motor.setVelocity(0.5)

        gather_data(command_func, DURATION, SAVE_NAME)

    finally:
        topMotor.disable()
        bottomMotor.disable()
