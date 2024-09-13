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
import numpy as np
from scipy.integrate import cumulative_trapezoid

Kp_pos = 10
Ki_pos = 0
Kp_vel = 10
Ki_vel = 0
SAVE_NAME = f'pos_C_chirp_test_Kpp = {Kp_pos} kpv = {Kp_vel} kiv = {Ki_vel}.csv'
DURATION = 10
WAVE_MAGNITUDE = 1
INITIAL_FREQUENCY = 0
FINAL_FREQUENCY = 25
CHIRP_RATE = (FINAL_FREQUENCY - INITIAL_FREQUENCY) / DURATION
SAMPLE_PERIOD = 0.04

integral_error = 0.0
previous_error = 0.0

f0 = 0
k = 2.5

def get_position_ref(running_time: float):
    t = np.arange(0, 0.01, running_time)
    velocity_ref = np.sin(2*np.pi * (f0*t + 0.5*k*t**2))
    position_ref = cumulative_trapezoid(velocity_ref, t, initial=0)
    return position_ref

def get_velocity_ref(running_time: float):
    return 



def pid_controller(error, integral_error, previous_error, dt):
    P = Kp_pos * error
    I = Ki_pos * integral_error
    # D = Kd * (error - previous_error) / dt
    return P + I 


def calculate_refPosition(running_time: float):
    return sum(
        coef * (running_time**i)
        for i, coef in enumerate()
    )

def calculate_refVelocity(running_time: float):
    return sum(
        coef * (running_time**i) for i, coef in enumerate()
    )

def get_frequency(t):
    return CHIRP_RATE * t + INITIAL_FREQUENCY
        
class State():
    def __init__(self):
        self.log = CVPPlot()
        self.csvwriter: CSVWriter | None = None
        self.initialised = False

def reset_position(rightKnee: RightKneeExoMotor, runningTime: float):
    if rightKnee.motor.getCVP().position > 0.1:
        rightKnee.motor.setVelocity(-0.5)
    elif rightKnee.motor.getCVP().position < -0.1:
        rightKnee.motor.setVelocity(0.5)
    else:
        rightKnee.motor.setPosition(0)

if __name__ == "__main__":
    
    state = State()

    def func(rightKnee: RightKneeExoMotor, runningTime: float):
        if not state.initialised:
            state.csvwriter = CSVWriter(SAVE_NAME, [rightKnee.motor])
            state.initialised = True

        frequency = get_frequency(runningTime)
        angular_frequency = 2 * np.pi * frequency
        position_ref = WAVE_MAGNITUDE * np.sin(angular_frequency * runningTime)

        cvp = rightKnee.motor.getCVP()

        rightKnee.motor.setPosition(position_ref)

        state.log.addCVP(runningTime, cvp)
        state.csvwriter.addCVP(runningTime, [rightKnee.motor])

    setup_teardown_motor_combination(RightKneeExoMotor(), func, DURATION)
    state.log.download(SAVE_NAME)
    setup_teardown_motor_combination(RightKneeExoMotor(), reset_position, 3)

    state.log.plot()


if __name__ == "__main__":
    
    state = State()


    def func(rightKnee: RightKneeExoMotor, runningTime: float):
        if not state.initialised:
            state.csvwriter = CSVWriter(SAVE_NAME, [rightKnee.motor])
            state.initialised = True


        velocity_ref = np.sin(2*np.pi * (f0*runningTime + 0.5*k*runningTime**2))
        position_ref = get_position_ref(runningTime)

        global integral_error, previous_error
        current_position = (
            rightKnee.motor.getCVP().position if rightKnee.motor.getCVP() is not None else 0.0
        )
        error = position_ref - current_position
        integral_error += error * SAMPLE_PERIOD
        velocity = pid_controller(
            error, integral_error, previous_error, SAMPLE_PERIOD
        ) + calculate_refVelocity(runningTime)
        rightKnee.motor.setVelocity(velocity)
        previous_error = error

        cvp = rightKnee.motor.getCVP()

        state.log.addCVP(runningTime, cvp)
        state.csvwriter.addCVP(runningTime, [rightKnee.motor])

    setup_teardown_motor_combination(RightKneeExoMotor(), func, DURATION)
    state.log.download(SAVE_NAME)
    setup_teardown_motor_combination(RightKneeExoMotor(), reset_position, 3)

    state.log.plot()





