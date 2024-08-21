import math
from aiosv2.CSVWriter import CSVWriter
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import CVPPlot

        
class State():
    def __init__(self):
        self.log = CVPPlot()
        self.csvwriter: CSVWriter | None = None
        pass

# def reset_position(runningTime: float, start_position):
#     return max(0, start_position - runningTime * (start_position))

CURRENT = 1.8
VELOCITY_MULT = 0.2

    
if __name__ == "__main__":

    state = State()
    def func(exoMotor: RightKneeExoMotor, runningTime: float):
        # if state.csvwriter is None:
        #     # state.csvwriter = CSVWriter(f"lowerleg{format(round(CURRENT, 2), '.2f')}.csv", [exoMotor.motor])
        #     state.csvwriter = CSVWriter(f"stop.csv", [exoMotor.motor])
        # exoMotor.motor.setCurrent(CURRENT)
        exoMotor.motor.setVelocity(1.8)

        cvp = exoMotor.motor.getCVP()
        
        state.log.addCVP(runningTime, cvp)
        # state.csvwriter.addCVP(runningTime, [exoMotor.motor])


    # # Reset the motor's position to zero
    # def func1(exoMotor: RightKneeExoMotor, runningTime: float):
    #     if exoMotor.getCVP() is None:
    #         return
    #     x1 = exoMotor.getCVP().position


    setup_teardown_rightknee_exomotor(func, 10)
    state.log.plot()
    