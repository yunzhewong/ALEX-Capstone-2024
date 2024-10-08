
import numpy as np
from ..aiosv2.SafeMotorOperation import SafeMotor
from dataGathering import gather_data


DURATION = 50
NAME = "chirp0to2550s.csv"

WAVE_MAGNITUDE = 3
INITIAL_FREQUENCY = 0
FINAL_FREQUENCY = 25
CHIRP_RATE = (FINAL_FREQUENCY - INITIAL_FREQUENCY) / DURATION

def get_frequency(t):
    return CHIRP_RATE * t + INITIAL_FREQUENCY

if __name__ == "__main__":
    def command_func(connection: SafeMotor, runningTime: float):
        frequency = get_frequency(runningTime)
        angular_frequency = 2 * np.pi * frequency
        current = WAVE_MAGNITUDE * np.sin(angular_frequency * runningTime)
        connection.setCurrent(current)   
        
    gather_data(command_func, DURATION, NAME)