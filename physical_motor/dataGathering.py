import math
import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time

DURATION = 3
NAME = "constantCurrent.csv"


def func(connection: aiosv2.ConnectedMotor, t):
    connection.setCurrent(1)


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
        func(connection, timeSinceStart)
        cvp = connection.getCVP()
        dataLog.addCVP(timeSinceStart, cvp)
    twinMotor.disable()

    dataLog.plot()
    dataLog.download(NAME)
