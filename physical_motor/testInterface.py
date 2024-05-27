import aiosv2
import time
import math
import numpy as np

if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    DURATION = 5
    SEND_FREQUENCY_HZ = 100
    SEND_PERIOD = 1 / SEND_FREQUENCY_HZ
    AMPLITUDE = 5

    # Version 1:
    t = np.arange(0, DURATION, SEND_PERIOD)
    sinusoidal_currents = AMPLITUDE * np.sin(2 * np.pi * t)
    for current in sinusoidal_currents:
        twinMotor.bottomMotor.setCurrent(current)
        time.sleep(0.01)
    twinMotor.bottomMotor.setCurrent(0)

    time.sleep(3)

    startTime = time.time()
    currentTime = startTime
    while currentTime - startTime < DURATION:
        currentTime = time.time()
        timeSinceStart = currentTime - startTime
        value = AMPLITUDE * np.sin(2 * np.pi * timeSinceStart)
        twinMotor.bottomMotor.setCurrent(value)
        time.sleep(0.01)

    twinMotor.bottomMotor.setCurrent(0)

    # Generate sinusoidal current input

    twinMotor.disable()
