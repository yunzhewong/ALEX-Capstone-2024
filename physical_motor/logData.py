import math
import time
import aiosv2
import numpy as np
import matplotlib.pyplot as plt
from classes.DataLog import DataLog

SAMPLE_PERIOD = 0.01
# SAVE_NAME = "lower_motor_pi_8.csv"
SAVE_NAME = "jerk_motor.csv"

## For jerk trajectory (6-degree)
POLYNOMIAL_COEFFICIENTS = [1e-6, -1e-5, 1e-4, -1e-3, 1e-2, -1e-1, 1]

# ## For sinusoidal waveform
# AMPLITUDE = 1.5  # Amplitude of the sinusoidal waveform (in amps)
# OFFSET = 0
# FREQUENCY_SIGNAL = 0.5  # Frequency of the sinusoidal waveform (in Hz)


# Store currents, velocities, positions, and a count of logged data points


# in this code, the motor actuates to pi / 8, 0, -pi/8, and back to 0
# prints the logged data across current, velocity, and position
if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    time.sleep(1)

    connection = twinMotor.bottomMotor

    WAITING_TIME_S = 15
    dataLog = DataLog()

    ## For polynomial current ##
    # Generate polynomial current input for three phases
    t = np.arange(0, 2, SAMPLE_PERIOD)
    phase1 = np.polyval(POLYNOMIAL_COEFFICIENTS, t)  # Move in one direction
    phase2 = -np.polyval(POLYNOMIAL_COEFFICIENTS, t)  # Move in the opposite direction

    # Define number of cycles
    num_cycles = 5

    # Repeat phases to create the desired number of cycles
    polynomial_currents = np.tile(np.concatenate((phase1, phase2)), num_cycles)

    # Loop to set polynomial current input
    start_time = time.time()
    for current in polynomial_currents:
        # Set current
        connection.setCurrent(current)

        # Log data
        cvp = connection.getCVP()
        current_time = time.time() - start_time
        dataLog.addCVP(current_time, cvp)

        time.sleep(SAMPLE_PERIOD)

    # ## Generate sinusoidal current input
    # t = np.arange(0, WAITING_TIME_S, SAMPLE_PERIOD)
    # sinusoidal_currents = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY_SIGNAL * t) + OFFSET

    # # Loop to set sinusoidal current input
    # start_time = time.time()
    # for i, current in enumerate(sinusoidal_currents):
    #     # Set current
    #     connection.setCurrent(current)

    #     # Log data
    #     cvp = connection.getCVP()
    #     current_time = time.time() - start_time
    #     dataLog.addCVP(current_time, cvp)

    #     time.sleep(SAMPLE_PERIOD)

    twinMotor.disable()

    dataLog.plot()
    dataLog.download(SAVE_NAME)
