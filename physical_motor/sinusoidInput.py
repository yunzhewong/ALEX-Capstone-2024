import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time

from aiosv2.TwinMotor import setup_teardown_twin_motor
from dataGathering import gather_data

if __name__ == "__main__":
    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        current = 3 * np.sin(2 * np.pi * runningTime)
        connection.setCurrent(current)    

    gather_data(command_func, 10, "sinusoid.csv")

    
