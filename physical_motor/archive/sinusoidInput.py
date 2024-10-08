import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time

from aiosv2.TwinMotor import setup_teardown_twin_motor
from dataGathering import gather_data


FREQUENCY = 1
MAGNITUDE = 1
DURATION = 10

if __name__ == "__main__":
    def command_func(topMotor: aiosv2.SafeMotor, bottomMotor: aiosv2.SafeMotor, runningTime: float):
        current = MAGNITUDE * np.sin(2 * np.pi * FREQUENCY * runningTime)
        topMotor.setVelocity(current)    

    gather_data(command_func, DURATION, f"sinusoid{FREQUENCY}Hz{MAGNITUDE}A.csv")

    
