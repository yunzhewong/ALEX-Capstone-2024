import math

import numpy as np
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.CSVWriter import CSVWriter
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import CVPPlot
from aiosv2 import aios
import trajectory

DURATION = 1
NAME = "chirp0to2550s.csv"

WAVE_MAGNITUDE = 2.5
INITIAL_FREQUENCY = 0
FINAL_FREQUENCY = 25
CHIRP_RATE = (FINAL_FREQUENCY - INITIAL_FREQUENCY) / DURATION

def get_frequency(t):
    return CHIRP_RATE * t + INITIAL_FREQUENCY
        
class State():
    def __init__(self):
        self.log = CVPPlot()
        self.csvwriter: CSVWriter | None = None
        self.initialised = False
    
if __name__ == "__main__":
    state = State()

    def func(rightKnee: RightKneeExoMotor, runningTime: float):
        if not state.initialised:
            state.csvwriter = CSVWriter("renamed.5A.csv", [rightKnee.motor])
            state.initialised = True

        frequency = get_frequency(runningTime)
        angular_frequency = 2 * np.pi * frequency
        current = WAVE_MAGNITUDE * np.sin(angular_frequency * runningTime)

        cvp = rightKnee.motor.getCVP()

        rightKnee.motor.setCurrent(current)

        state.log.addCVP(runningTime, cvp)
        state.csvwriter.addCVP(runningTime, [rightKnee.motor])


    setup_teardown_motor_combination(RightKneeExoMotor(), func, 1)
    

    state.log.plot()
    