from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import DataLog

        
class State():
    def __init__(self):
        self.log = DataLog()
        pass

    
if __name__ == "__main__":

    state = State()
    def func(exoMotor: RightKneeExoMotor, runningTime: float):
        exoMotor.motor.setPosition(runningTime)
        cvp = exoMotor.motor.getCVP()

        if cvp is None:
            return
        state.log.addCVP(runningTime, cvp, CVP(0, 0, 0))

    setup_teardown_rightknee_exomotor(func, 20)
    state.log.plot()
    