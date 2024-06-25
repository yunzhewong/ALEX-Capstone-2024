import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time

from aiosv2.TwinMotor import setup_teardown_twin_motor

if __name__ == "__main__":
    dataLog = DataLog()

    def func(twinMotor: aiosv2.TwinMotor, runningTime: float):
        connection = twinMotor.bottomMotor
        current = 2 * np.sin(2 * np.pi * runningTime)
        connection.setCurrent(current)
        cvp = connection.getCVP()
        dataLog.addCVP(runningTime, cvp)

    setup_teardown_twin_motor(func, 15)

    dataLog.plot()
    dataLog.download("sinusoid.csv")
    
