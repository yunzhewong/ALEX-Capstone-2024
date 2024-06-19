
import numpy as np
import aiosv2
from dataGathering import gather_data


DURATION = 5
NAME = "chirp025.csv"

WAVE_MAGNITUDE = 2
INITIAL_FREQUENCY = 0
FINAL_FREQUENCY = 25
CHIRP_RATE = (FINAL_FREQUENCY - INITIAL_FREQUENCY) / DURATION

def get_frequency(t):
    frequency_slope = CHIRP_RATE
    return frequency_slope * t + INITIAL_FREQUENCY


def func(connection: aiosv2.ConnectedMotor, t):
    frequency = get_frequency(t)
    angular_frequency = 2 * np.pi * frequency
    current = WAVE_MAGNITUDE * np.sin(angular_frequency * t)
    connection.setCurrent(current)

if __name__ == "__main__":
    gather_data(func, DURATION, NAME)
    
