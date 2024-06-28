import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time

from aiosv2.TwinMotor import setup_teardown_twin_motor
from dataGathering import gather_data


FREQUENCY = 1
MAGNITUDE = 2
DURATION = 10

if __name__ == "__main__":
    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        current = MAGNITUDE * np.sin(2 * np.pi * FREQUENCY * runningTime)
        connection.setCurrent(current)    

    gather_data(command_func, DURATION, f"sinusoid{FREQUENCY}Hz{MAGNITUDE}A.csv")

    
