import math
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.CSVWriter import CSVWriter
from aiosv2.RightKneeExoMotor import RightKneeExoMotor
from classes.DataLog import CVPPlot
import numpy as np
import matplotlib.pyplot as plt
from rebuild import EXO24Trajectory
import trajectory as t

DURATION = 86
SAMPLE_PERIOD = 1/300

CONTROLLER_P_GAIN = 10
CONTROLLER_I_GAIN = 0.1

class PIController():
    def __init__(self, Kp: float, Ki: float):
        self.Kp = Kp
        self.Ki = Ki
        self.integral_error = 0
    
    def compute_control(self, error: float, dt: float):
        self.integral_error += error * dt
        return self.Kp * error + self.Ki * self.integral_error  

class State():
    def __init__(self):
        self.log = CVPPlot()
        self.last_time = 0
        self.initialised = False

if __name__ == "__main__":
    trajectory = EXO24Trajectory.Exo24Trajectory(SAMPLE_PERIOD)
    controller = PIController(CONTROLLER_P_GAIN, CONTROLLER_I_GAIN)

    state = State()

    def main(rightKnee: RightKneeExoMotor, runningTime: float):
        reference_position, reference_velocity = trajectory.get_state(runningTime)

        INDEX = 1
        reference_motor_position = reference_position[INDEX]
        reference_motor_velocity = reference_velocity[INDEX]

        cvp = rightKnee.motor.getCVP()

        dt = runningTime - state.last_time

        position_error = reference_motor_position - cvp.position
        position_correction = controller.compute_control(position_error, dt)
        motor_velocity = reference_motor_velocity + position_correction

        state.last_time = runningTime

        rightKnee.motor.setVelocity(motor_velocity)

        state.log.addCVP(runningTime, cvp)

    setup_teardown_motor_combination(RightKneeExoMotor(), main, DURATION)

    def reset(rightKnee: RightKneeExoMotor, runningTime: float):
        t.move_to_pos(rightKnee.motor, 0, 0.3)
        
    setup_teardown_motor_combination(RightKneeExoMotor(), reset, 10)

    state.log.plot()







