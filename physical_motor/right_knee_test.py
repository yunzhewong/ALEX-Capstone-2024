import math
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import CVPPlot

        
class State():
    def __init__(self):
        self.log = CVPPlot()
        pass

    
if __name__ == "__main__":

    state = State()
    def func(exoMotor: RightKneeExoMotor, runningTime: float):
        exoMotor.motor.setVelocity(-2)

        cvp = exoMotor.motor.getCVP()
        state.log.addCVP(runningTime, cvp)


    setup_teardown_rightknee_exomotor(func, 10)
    state.log.plot()
    