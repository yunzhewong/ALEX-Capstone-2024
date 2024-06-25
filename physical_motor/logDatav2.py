import math
import time
import aiosv2
import numpy as np
import matplotlib.pyplot as plt
from classes.DataLog import DataLog
from dataGathering import gather_data

# SAVE_NAME = "lower_motor_pi_8.csv"
SAVE_NAME = "polynomial_position_motor.csv"
DURATION = 15

## For jerk trajectory (6-degree in position)
SAMPLE_PERIOD = 0.04
POLYNOMIAL_COEFFICIENTS = [0.01, 0.1, 0.01, 0.01]

# ## For sinusoidal waveform
# AMPLITUDE = 1.5  # Amplitude of the sinusoidal waveform (in amps)
# OFFSET = 0
# FREQUENCY_SIGNAL = 0.5  # Frequency of the sinusoidal waveform (in Hz)


# Store currents, velocities, positions, and a count of logged data points


# in this code, the motor actuates to pi / 8, 0, -pi/8, and back to 0
# prints the logged data across current, velocity, and position

def get_poly_time(runningTime: float):
    CYCLE_DURATION = 4.0  # Duration of each cycle (2 seconds in each direction)

    cycle_time = runningTime % CYCLE_DURATION

    if cycle_time > CYCLE_DURATION / 2:
        return CYCLE_DURATION - cycle_time
    return cycle_time


if __name__ == "__main__":

    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        poly_time = get_poly_time(runningTime) 
        position = np.polyval(POLYNOMIAL_COEFFICIENTS, poly_time)
        connection.setPosition(position)


    time.sleep(1)
    gather_data(command_func, DURATION, SAVE_NAME)

