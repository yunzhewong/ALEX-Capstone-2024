import math
import time
import aiosv2
import numpy as np
import matplotlib.pyplot as plt
from classes.DataLog import DataLog

# SAVE_NAME = "lower_motor_pi_8.csv"
SAVE_NAME = "polynomial_position_motor.csv"

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
if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    time.sleep(1)

    connection = twinMotor.bottomMotor

    WAITING_TIME_S = 15
    dataLog = DataLog()

    ## For polynomial current ##
    # Generate position input
    t = np.arange(0, 2, SAMPLE_PERIOD)
    position = np.polyval(POLYNOMIAL_COEFFICIENTS, t)

    start_time = time.time()
    start_time_actual = start_time
    cycle_duration = 4.0  # Duration of each cycle (2 seconds in each direction)

    while time.time() - start_time_actual < WAITING_TIME_S:
        #print(position)
        for i in range(len(position)):
            # Set position
            pos = position[i]
            #print(pos)
            connection.setPosition(pos)

            # Log data
            cvp = connection.getCVP()
            current_time = time.time() - start_time_actual
            dataLog.addCVP(current_time, cvp)
            
            if (i == len(position)-1):
                position = [position[-1-i] for i in range(len(position))]
                start_time = time.time()
                break
                #time.sleep(max(cycle_duration/2-(time.time()-start_time),0))
            

            # # Check if it's time to reverse direction
            # if time.time() - start_time >= cycle_duration/2:
                
            #     position = [position[-1-i] for i in range(len(position))]
            #     #POLYNOMIAL_COEFFICIENTS[-1] *= -1  # Reverse direction
            #     start_time = time.time()  # Reset start time for next cycle
            #     break  # Break out of the inner loop to switch direction
            
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
