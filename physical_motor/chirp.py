
import numpy as np
import aiosv2
from dataGathering import gather_data


DURATION = 25
NAME = "chirp25to50.csv"

WAVE_MAGNITUDE = 5
INITIAL_FREQUENCY = 25
FINAL_FREQUENCY = 50
CHIRP_RATE = (FINAL_FREQUENCY - INITIAL_FREQUENCY) / DURATION

def get_frequency(t):
    return CHIRP_RATE * t + INITIAL_FREQUENCY

if __name__ == "__main__":
    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        frequency = get_frequency(runningTime)
        angular_frequency = 2 * np.pi * frequency
        current = WAVE_MAGNITUDE * np.sin(angular_frequency * runningTime)
        connection.setCurrent(current)   
        
    gather_data(command_func, DURATION, NAME)