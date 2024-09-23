from aiosv2.CSVWriter import CSVWriter
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.Controllers import CascadeController
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.RightKneeExoMotor import RightKneeExoMotor
from classes.DataLog import CVPPlot
from rebuild import EXO24Trajectory
import trajectory as t

DURATION = 86

class State():
    def __init__(self):
        self.log = CVPPlot()
        self.kneelog: CSVWriter | None = None
        self.extlog: CSVWriter | None = None
        self.abdlog: CSVWriter | None = None
        self.last_time = 0
        self.initialised = False

if __name__ == "__main__":
    trajectory = EXO24Trajectory.Exo24Trajectory()

    leftAbductorController = CascadeController()
    rightAbductorController = CascadeController()
    leftExtensorController = CascadeController()
    rightExtensorController = CascadeController()
    leftKneeController = CascadeController()
    rightKneeController = CascadeController()

    state = State()

    def func(exoskeleton: Exoskeleton, runningTime: float):
        if not state.initialised:
            state.kneelog = CSVWriter("knee2.csv", [exoskeleton.rightKnee])
            state.extlog = CSVWriter("extens2.csv", [exoskeleton.rightExtensor])
            state.abdlog = CSVWriter("abduct2.csv", [exoskeleton.rightAbductor])
            state.initialised = True

        reference_position, reference_velocity = trajectory.get_state(runningTime)

        dt = runningTime - state.last_time

        leftAbductorController.set_reference(exoskeleton.leftAbductor, reference_position[0], reference_velocity[0], dt)
        rightAbductorController.set_reference(exoskeleton.rightAbductor, reference_position[1], reference_velocity[1], dt)
        leftExtensorController.set_reference(exoskeleton.leftExtensor, reference_position[2], reference_velocity[2], dt)
        rightExtensorController.set_reference(exoskeleton.rightExtensor, reference_position[3], reference_velocity[3], dt)
        leftKneeController.set_reference(exoskeleton.leftKnee, reference_position[4], reference_velocity[4], dt)
        rightKneeController.set_reference(exoskeleton.rightKnee, reference_position[5], reference_velocity[5], dt)

        state.last_time = runningTime
        
        cvp = exoskeleton.rightKnee.getCVP()
        state.log.addCVP(runningTime, cvp)
        state.kneelog.addCVP(runningTime, [exoskeleton.rightKnee])
        state.extlog.addCVP(runningTime, [exoskeleton.rightExtensor])
        state.abdlog.addCVP(runningTime, [exoskeleton.rightAbductor])

    setup_teardown_motor_combination(Exoskeleton(), func, DURATION)

    state.log.plot()







