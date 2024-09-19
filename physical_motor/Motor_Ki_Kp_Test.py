import math
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.CSVWriter import CSVWriter
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import CVPPlot
from aiosv2 import aios
import numpy as np
from scipy.integrate import cumulative_trapezoid
from rebuild import trajectoryGenerator as tg
import matplotlib.pyplot as plt
from rebuild.EXO24Trajectory import Exo24Trajectory

Kp_pos = 10
Ki_pos = 0.1
Kp_vel = 10
Ki_vel = 0
SAVE_NAME = f'Cascade_traj_kpp={Kp_pos}_kip={Ki_pos}_kpv={Kp_vel}_kiv={Ki_vel}.csv'
DURATION = 60
WAVE_MAGNITUDE = 1
INITIAL_FREQUENCY = 0
FINAL_FREQUENCY = 25
CHIRP_RATE = (FINAL_FREQUENCY - INITIAL_FREQUENCY) / DURATION
SAMPLE_PERIOD = 0.04
DESIRED_VELOCITY = [0.0, 2*(3 * math.pi / 200), 3*(-1 * math.pi / 1000)]
DESIRED_TRAJECTORY = [0.0, 0.0, (3 * math.pi / 200), (-1 * math.pi / 1000)]

POLY_COEFFICIENTS = [
    0.0,
    2*(3 * math.pi / 200),
    3*(-1 * math.pi / 1000),
]
POLYNOMIAL_COEFFICIENTS = [
    0.0,
    0.0,
    (3 * math.pi / 200),
    (-1 * math.pi / 1000),
]

integral_error = 0.0
previous_error = 0.0

f0 = 0
k = 2.5

def get_position_ref(running_time: float):
    velocity_ref = np.sin(2*np.pi * (f0*running_time + 0.5*k*running_time**2))
    position_ref = cumulative_trapezoid(velocity_ref, running_time, initial=0)
    return position_ref

def get_velocity_ref(running_time: float):
    return np.sin(2*np.pi * (f0*running_time + 0.5*k*running_time**2))



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
        coef * (running_time**i) for i, coef in enumerate(POLY_COEFFICIENTS)
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
    


# if __name__ == "__main__":
    
#     state = State()
    
#     # trajectory_positions, trajectory_velocities = create_trajectory(SAMPLE_PERIOD)
#     # trajectory_positions = trajectory_positions[5,:]
#     # trajectory_velocities = trajectory_velocities[5,:]

#     def func(rightKnee: RightKneeExoMotor, runningTime: float):
#         if not state.initialised:
#             state.csvwriter = CSVWriter(SAVE_NAME, [rightKnee.motor])
#             state.initialised = True
            
#         # n_steps = trajectory_velocities.shape[1]  # Number of discretized time steps
#         # trajectory_time = np.linspace(0, (n_steps - 1) * SAMPLE_PERIOD, n_steps)
#         # closest_index = np.argmin(np.abs(trajectory_time - runningTime))
#         # velocity_ref = trajectory_velocities[:, closest_index]
#         # position_ref = trajectory_positions[:, closest_index]

#         velocity_ref = sum(coef * (runningTime**i) for i, coef in enumerate(POLY_COEFFICIENTS))

#         # position_ref = WAVE_MAGNITUDE * np.sin(angular_frequency * runningTime)
#         cvp = rightKnee.motor.getCVP()

#         rightKnee.motor.setVelocity(velocity_ref)

#         state.log.addCVP(runningTime, cvp)
#         state.csvwriter.addCVP(runningTime, [rightKnee.motor])

#     setup_teardown_motor_combination(RightKneeExoMotor(), func, DURATION)
#     state.log.download(SAVE_NAME)
#     setup_teardown_motor_combination(RightKneeExoMotor(), reset_position, 3)

#     state.log.plot()

def get_value(time_array, value_array, running_time):
    index = np.argmin(np.abs(time_array - running_time))
    return value_array[index]


if __name__ == "__main__":

    trajectory = Exo24Trajectory()
    
    state = State()


    # fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

    # ax1.plot(time_array_vel, right_knee_velocities)
    # ax2.plot(time_array_pos, right_knee_positions)
    # plt.show()




    def func(rightKnee: RightKneeExoMotor, runningTime: float):
        if not state.initialised:
            state.csvwriter = CSVWriter(SAVE_NAME, [rightKnee.motor])
            state.initialised = True

        positions, velocities = trajectory.get_state(runningTime)

        velocity_ref = velocities[5]
        position_ref = positions[5]

        global integral_error, previous_error
        cvp = rightKnee.motor.getCVP()
        current_position = cvp.position

        error = position_ref - current_position
        integral_error += error * SAMPLE_PERIOD
        velocity = pid_controller(
            error, integral_error, previous_error, SAMPLE_PERIOD
        ) + velocity_ref
        # Incorporate desired trajectory and velocity

        # desired_position = 0
        # desired_velocity = 0

        # Adjust control outputs based on desired values
        # velocity += desired_velocity - calculate_refVelocity(runningTime)
        # print(velocity)
        rightKnee.motor.setVelocity(velocity)
        previous_error = error

        state.log.addCVP(runningTime, cvp)
        state.csvwriter.addCVP(runningTime, [rightKnee.motor])

    setup_teardown_motor_combination(RightKneeExoMotor(), func, DURATION)
    state.log.download(SAVE_NAME)
    setup_teardown_motor_combination(RightKneeExoMotor(), reset_position, 2)

    state.log.plot()





