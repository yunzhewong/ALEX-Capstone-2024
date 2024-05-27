import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time

DURATION = 10

if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    connection = twinMotor.bottomMotor

    dataLog = DataLog()

    startTime = time.time()
    currentTime = startTime
    while currentTime - startTime < DURATION:
        currentTime = time.time()
        timeSinceStart = currentTime - startTime
        current = 2 * np.sin(2 * np.pi * timeSinceStart)
        connection.setCurrent(current)
        cvp = connection.getCVP()
        dataLog.addCVP(timeSinceStart, cvp)
        time.sleep(0.01)

    twinMotor.disable()

    dataLog.plot()
    dataLog.download("sinusoid")
