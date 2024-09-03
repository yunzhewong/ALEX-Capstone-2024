import math
from aiosv2.CSVWriter import CSVWriter
from classes.DataLog import CVPPlot
import trajectory
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.ControlLoop import setup_teardown_motor_combination

FALL_TIME = 25

class State():
    def __init__(self):
        self.log = CVPPlot()
        self.csvwriter: CSVWriter | None = None
        self.initialised = False
    

if __name__ == "__main__":
    state = State()

    def func(exoskeleton: Exoskeleton, runningTime: float):
        if not state.initialised:
            state.csvwriter = CSVWriter("blank.csv", [exoskeleton.rightAbductor])
            state.initialised = True
        trajectory.slow_move_to_pos(exoskeleton.rightAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftExtensor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftKnee, 0)
        trajectory.slow_move_to_pos(exoskeleton.rightKnee, 0)

        if (runningTime < FALL_TIME):
            trajectory.slow_move_to_pos(exoskeleton.rightAbductor, -math.pi / 2)
        else:
            exoskeleton.rightAbductor.setCurrent(0)

        cvp = exoskeleton.rightAbductor.getCVP()
        state.log.addCVP(runningTime, cvp)
        state.csvwriter.addCVP(runningTime, [exoskeleton.rightAbductor])
        
    setup_teardown_motor_combination(Exoskeleton(), func, 50)

    state.log.plot()