from aiosv2.Controllers import CascadeController
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.RightKneeExoMotor import RightKneeExoMotor
from classes.DataLog import CVPPlot
from rebuild import EXO24Trajectory

DURATION = 86
SAMPLE_PERIOD = 1/300

class State():
    def __init__(self):
        self.log = CVPPlot()
        self.last_time = 0
        self.initialised = False

if __name__ == "__main__":
    trajectory = EXO24Trajectory.Exo24Trajectory(SAMPLE_PERIOD)
    controller = CascadeController()

    state = State()

    def func(rightKnee: RightKneeExoMotor, runningTime: float):
        reference_position, reference_velocity = trajectory.get_state(runningTime)

        dt = runningTime - state.last_time

        INDEX = 5
        controller.set_reference(rightKnee.motor, reference_position[INDEX], reference_velocity[INDEX], dt)

        state.last_time = runningTime
        
        cvp = rightKnee.motor.getCVP()
        state.log.addCVP(runningTime, cvp)

    setup_teardown_motor_combination(RightKneeExoMotor(), func, DURATION)

    state.log.plot()







