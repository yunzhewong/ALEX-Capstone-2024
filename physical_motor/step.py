import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time

from aiosv2.TwinMotor import setup_teardown_twin_motor
from dataGathering import gather_data

MAGNITUDE = 0.8
DURATION = 5

if __name__ == "__main__":
    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        connection.setCurrent(MAGNITUDE)    

    gather_data(command_func, DURATION, f"step{MAGNITUDE}A.csv")

    
