from aiosv2.DemoBaseMotor import DemoBaseMotor
from aiosv2.TwinMotor import TwinMotor
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.Controllers import CascadeController
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.RightKneeExoMotor import RightKneeExoMotor
from classes.DataLog import RealTimePlot
from rebuild import EXO24Trajectory

DURATION = 86

class State():
    def __init__(self):
        self.plot = RealTimePlot(DURATION)
        self.last_time = 0

if __name__ == "__main__":
    state = State()
    trajectory = EXO24Trajectory.Exo24Trajectory()
    controller = CascadeController()

    def func(demoBaseMotor: DemoBaseMotor, runningTime: float):
        dt = runningTime - state.last_time
        positions, velocities = trajectory.get_state(runningTime)

        controller.set_reference(demoBaseMotor.motor, positions[5], velocities[5], dt)
        cvp = demoBaseMotor.motor.getCVP()
        state.last_time = runningTime
        try:
            state.plot.addReading(runningTime, cvp, ref_velocity=velocities[5], ref_position=positions[5])
        except:
            raise Exception("Plot Error")

    setup_teardown_motor_combination(DemoBaseMotor(), func, DURATION)

    state.plot.wait_for_interrupt()









