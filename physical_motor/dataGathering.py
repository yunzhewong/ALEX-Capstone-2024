import math
import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time


def gather_data(func, duration, name):
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    connection = twinMotor.bottomMotor

    dataLog = DataLog()

    startTime = time.time()
    currentTime = startTime
    while currentTime - startTime < duration:
        currentTime = time.time()
        timeSinceStart = currentTime - startTime
        func(connection, timeSinceStart)
        cvp = connection.getCVP()
        dataLog.addCVP(timeSinceStart, cvp)
    twinMotor.disable()

    dataLog.plot()
    dataLog.download(name)





    
