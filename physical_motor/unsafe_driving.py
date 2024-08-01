from typing import List
from aiosv2.CVP import CVP
import aiosv2
from aiosv2.TwinMotor import TwinMotor, setup_teardown_twin_motor
import time
import math

from dataGathering import gather_data
from classes.DataLog import DataLog



if __name__ == "__main__":

    log = DataLog()
    def func(twinMotor: TwinMotor, runningTime: float):
        twinMotor.topMotor.setVelocity(3)
        cvp = twinMotor.topMotor.getCVP()

        log.addCVP(runningTime, cvp, CVP(0,0,0))
    setup_teardown_twin_motor(func, 5)

    log.plot()
    