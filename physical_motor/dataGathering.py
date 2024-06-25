from typing import Callable
import aiosv2
from classes.DataLog import DataLog
import numpy as np
import time
from aiosv2.TwinMotor import setup_teardown_twin_motor

def gather_data(command_func: Callable[[aiosv2.SafeMotor, float], None], duration: float, name: str):
    dataLog = DataLog()

    def func(twinMotor: aiosv2.TwinMotor, runningTime: float):
        connection = twinMotor.bottomMotor
        command_func(connection, runningTime)
        cvp = connection.getCVP()
        dataLog.addCVP(runningTime, cvp)

    setup_teardown_twin_motor(func, duration)

    dataLog.plot()
    dataLog.download(name)
    


    
