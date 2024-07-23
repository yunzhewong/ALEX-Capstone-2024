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


def reset_position(motor:SafeMotor):

    if motor.getCVP().position < 0:
        motor.setVelocity(0.5)
    else:
        motor.setVelocity(0)




# def return_to_start_position():

if __name__ == "__main__":
    try:
        socket = AiosSocket()
        config = SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=1*math.pi, minimum_position=-2 * math.pi / 3, maximum_position=2 * math.pi / 3)
        topMotor = SafeMotor("10.10.10.16", socket, config)
        bottomMotor = SafeMotor("10.10.10.17", socket, config)

        topMotor.enable()
        bottomMotor.enable()

        time.sleep(1)

        def command_func(top_motor: SafeMotor, bottom_motor:SafeMotor, running_time):

            bottom_motor.setPosition(0)

            if (running_time <= 5):
                top_motor.setCurrent(-5)

            if (running_time > 5):
                reset_position(top_motor)


        gather_data(command_func, DURATION, SAVE_NAME)

    finally:
        topMotor.disable()
        bottomMotor.disable()


    # try:
    #     socket = AiosSocket()
    #     config = SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=4*math.pi, minimum_position=-2 * math.pi / 3, maximum_position=2 * math.pi / 3)
    #     topMotor = SafeMotor("10.10.10.16", socket, config)
    #     bottomMotor = SafeMotor("10.10.10.17", socket, config)

    #     topMotor.enable()
    #     bottomMotor.disable()

    #     time.sleep(1)

    #     def command_func(top_motor: SafeMotor, bottom_motor:SafeMotor, running_time):
    #         duration = DURATION 
    #         if running_time <= duration:
    #             top_motor.setVelocity(calculate_velocity(1))
    #             # top_motor.setVelocity(calculate_velocity(-20))
        

    #     gather_data(command_func, DURATION, SAVE_NAME)

    # finally:
    #     topMotor.disable()
    #     bottomMotor.disable()