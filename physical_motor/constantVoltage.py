import math
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from aiosv2.CSVWriter import CSVWriter
from classes.DataLog import CVPPlot

        
class State():
    def __init__(self):
        self.plot = CVPPlot()
        self.log: CSVWriter | None = None
        pass

DELTA = 0.15
    
if __name__ == "__main__":

    state = State()
    def func(exoMotor: RightKneeExoMotor, runningTime: float):
        if state.log is None:
            state.log = CSVWriter("delta0.1.csv", [exoMotor.motor])
        cvp = exoMotor.motor.getCVP()
        exoMotor.motor.setPosition(cvp.position + DELTA)

        state.log.addCVP(runningTime, [exoMotor.motor])
        state.plot.addCVP(runningTime, cvp)


    setup_teardown_rightknee_exomotor(func, 4)
    state.plot.plot()
    