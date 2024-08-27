import math
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.CSVWriter import CSVWriter
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import CVPPlot
from aiosv2 import aios
import trajectory

        
class State():
    def __init__(self):
        self.log = CVPPlot()
        self.csvwriter: CSVWriter | None = None
        self.initialised = False
    
if __name__ == "__main__":
    state = State()

    def func(exoskeleton: Exoskeleton, runningTime: float):
        if not state.initialised:
            state.csvwriter = CSVWriter("0.2rads sinusoid right extensor.csv", [exoskeleton.rightExtensor])
            state.initialised = True

        trajectory.slow_move_to_pos(exoskeleton.rightAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftExtensor, 0)
        trajectory.slow_move_to_pos(exoskeleton.rightKnee, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftKnee, 0)


        cvp = exoskeleton.rightExtensor.getCVP()

        exoskeleton.rightExtensor.setVelocity(0.2 * math.sin(runningTime))
        
        state.log.addCVP(runningTime, cvp)
        state.csvwriter.addCVP(runningTime, [exoskeleton.rightExtensor])


    setup_teardown_motor_combination(Exoskeleton(), func, 20)
    

    state.log.plot()
    