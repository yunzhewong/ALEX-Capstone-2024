from enum import Enum
from aiosv2.DemoBaseMotor import DemoBaseMotor
from aiosv2.TwinMotor import TwinMotor
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.Controllers import CascadeController
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.RightKneeExoMotor import RightKneeExoMotor
from classes.DataLog import RealTimePlot
from rebuild import EXO24Trajectory

DURATION = 10

class DemoType(Enum):
     PositiveCurrent = 1
     NegativeCurrent = 2
     PositiveVelocity = 3
     NegativeVelocity = 4

demoType = DemoType.PositiveVelocity

CURRENT = 0.85
VELOCITY = 0.2

class State():
    def __init__(self):
        self.plot = RealTimePlot(DURATION)
        self.last_time = 0

if __name__ == "__main__":
    state = State()

    def func(demoBaseMotor: DemoBaseMotor, runningTime: float):
        cvp = demoBaseMotor.motor.getCVP()
        if (demoType == DemoType.PositiveVelocity):
            demoBaseMotor.motor.setVelocity(VELOCITY)
            state.plot.addReading(runningTime, cvp, ref_velocity=VELOCITY)
        elif (demoType == DemoType.NegativeVelocity):
            demoBaseMotor.motor.setVelocity(-VELOCITY)
            state.plot.addReading(runningTime, cvp, ref_velocity=-VELOCITY)
        elif (demoType == DemoType.PositiveCurrent):
            demoBaseMotor.motor.setCurrent(CURRENT)
            state.plot.addReading(runningTime, cvp)
        else:
            demoBaseMotor.motor.setCurrent(-1 * CURRENT)
            state.plot.addReading(runningTime, cvp)
            
    setup_teardown_motor_combination(DemoBaseMotor(), func, DURATION)

    state.plot.wait_for_interrupt()









