import math
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import DataLog

        
class State():
    def __init__(self):
        # self.log = DataLog()
        pass

    
if __name__ == "__main__":

    state = State()
    def func(exoMotor: RightKneeExoMotor, runningTime: float):
        target = math.floor(runningTime) * math.pi / 16
        exoMotor.motor.setPosition(target)
        # state.log.addCVP(runningTime, exoMotor.motor.getCVP(), CVP(0, 0, 0))

    setup_teardown_rightknee_exomotor(func, 20)
    # state.log.plot()
    