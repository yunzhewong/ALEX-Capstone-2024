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
from rebuild import trajectoryGenerator as tg

Kp_pos = 10
Ki_pos = 0.01
Kp_vel = 10
Ki_vel = 0
SAVE_NAME = f'pos_C_chirp_test_Kpp = {Kp_pos} kpv = {Kp_vel} kiv = {Ki_vel}.csv'
DURATION = 10
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
        

def create_trajectory(dt):
    viaPoints = []
    
    # Time, L-Abd, R-Abd, L-Ext, R-Ext, L-Knee, R-Knee
    # WIDEN HIPS
    viaPoints.append(np.array([0.0, -0.0, 0.0, 0.0, -0.0, -0.0, 0.0]))
    viaPoints.append(np.array([1.0, -6.0, 6.0, 0.0, -0.0, -0.0, 0.0]))
    
    # SIT TO STAND
    viaPoints.append(np.array([4.0, -6.0, 6.0, 45.0, -45.0, -90.0, 90.0]))
    viaPoints.append(np.array([5.0, -6.0, 6.0, 45.0, -45.0, -90.0, 90.0]))
    viaPoints.append(np.array([8.0, -6.0, 6.0, 0.0, -0.0, -0.0, 0.0]))

    init_time = 1 + 1 + 8
    GAIT_CYCLE_TIME = 12.0

    def get_time(start_time: float, gait_percentage: float):
        return start_time + gait_percentage * GAIT_CYCLE_TIME

    GAIT_CYCLE_POSITIONS_LENGTH = 21
    
    # Define waypoints for all joints
    Knee_R = [10.0, 15.0, 22.0, 25.0, 22.0, 15.0, 10.0, 7.0, 5.0, 7.0, 10.0, 20.0, 30.0, 45.0, 55.0, 60.0, 55.0, 40.0, 25.0, 15.0, 10.0]
    Ext_R = [-35.0, -34.0, -32.0, -30.0, -25.0, -17.0, -10.0, -5.0, -0.0, 5.0, 8.0, 10.0, 5.0, 0.0, -12.0, -25.0, -30.0, -33.0, -35.0, -37.0, -35.0,]
    Abd_R = [0.0, 1.0, 4.0, 7.0, 10.0, 7.0, 4.0, 2.0, 1.0, 0.0, 0.0, -1.0, -2.0, -2.0, -2.0, -1.0, -0.0, 0.0, 0.0, 1.0, 1.0,]

    OFFSET_LENGTH = math.floor(GAIT_CYCLE_POSITIONS_LENGTH / 2)

    Knee_L = -np.roll(Knee_R, OFFSET_LENGTH)
    Ext_L = -np.roll(Ext_R, OFFSET_LENGTH)
    Abd_L = -np.roll(Abd_R, OFFSET_LENGTH)

    assert len(Knee_R) == GAIT_CYCLE_POSITIONS_LENGTH
    assert len(Knee_L) == GAIT_CYCLE_POSITIONS_LENGTH
    assert len(Ext_R) == GAIT_CYCLE_POSITIONS_LENGTH
    assert len(Ext_L) == GAIT_CYCLE_POSITIONS_LENGTH
    assert len(Abd_R) == GAIT_CYCLE_POSITIONS_LENGTH
    assert len(Abd_L) == GAIT_CYCLE_POSITIONS_LENGTH

    ## Gait Cycles
    GAIT_CYCLES = 2
    for _ in range(GAIT_CYCLES):
        for i in range(0, GAIT_CYCLE_POSITIONS_LENGTH - 1):
            viaPoints.append(
                np.array(
                    [   get_time(init_time, i * 0.05),
                        -6 + Abd_L[i],
                        6 + Abd_R[i],
                        Ext_L[i],
                        Ext_R[i],
                        Knee_L[i],
                        Knee_R[i],
                    ]
                )
            )
        init_time = init_time + GAIT_CYCLE_TIME


    SIDE_STEP_CYCLES = 2
    for _ in range(SIDE_STEP_CYCLES):
        viaPoints.append(
            np.array([get_time(init_time, 0.05), -6.0, 6.0, 10.0, -10.0, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.15), -6, 6, 10.0, -30.0, -10.0, 65.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.30), -0, 30, 10.0, -30.0, -10.0, 65.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.50), -0, 30, 10.0, -10, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.65), -30, 0, 10.0, -10, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.80), -30, 0, 30.0, -10.0, -65.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.90), -6, 6, 30.0, -10.0, -65.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 1), -6, 6, 10.0, -10.0, -10.0, 10.0])
        )
        init_time = init_time + GAIT_CYCLE_TIME

    # RESET HIPS
    viaPoints.append(
        np.array([get_time(init_time, 0.10), -6.0, 6.0, 0.0, 0.0, -0.0, 0.0])
    )
    viaPoints.append(
        np.array([get_time(init_time, 0.20), -0.0, 0.0, 0.0, -0.0, -0.0, 0.0])
    )

    # Convert via points to trajectory
    vp = np.zeros([len(viaPoints), len(viaPoints[0])])
    for i in range(len(viaPoints)):
        vp[i] = viaPoints[i]
    
    return tg.getTrajectory(vp.T, dt, Kv=0.75)


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


if __name__ == "__main__":
    
    state = State()

    trajectory_positions, trajectory_velocities = create_trajectory(SAMPLE_PERIOD)
    right_knee_traj = trajectory_velocities[5,:]



    def func(rightKnee: RightKneeExoMotor, runningTime: float):
        if not state.initialised:
            state.csvwriter = CSVWriter(SAVE_NAME, [rightKnee.motor])
            state.initialised = True


        velocity_ref = sum(coef * (runningTime**i) for i, coef in enumerate(POLY_COEFFICIENTS))
        position_ref = sum(coef * (runningTime**i)for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS))

        global integral_error, previous_error
        current_position = (
            rightKnee.motor.getCVP().position if rightKnee.motor.getCVP() is not None else 0.0
        )
        error = position_ref - current_position
        integral_error += error * SAMPLE_PERIOD
        velocity = pid_controller(
            error, integral_error, previous_error, SAMPLE_PERIOD
        ) + calculate_refVelocity(runningTime)
        # Incorporate desired trajectory and velocity
        desired_position = sum(
            coef * (runningTime**i) for i, coef in enumerate(DESIRED_TRAJECTORY)
        )
        desired_velocity = sum(
            coef * (runningTime**i) for i, coef in enumerate(DESIRED_VELOCITY)
        )
        # Adjust control outputs based on desired values
        velocity += desired_velocity - calculate_refVelocity(runningTime)
        rightKnee.motor.setVelocity(velocity)
        previous_error = error

        cvp = rightKnee.motor.getCVP()

        state.log.addCVP(runningTime, cvp)
        state.csvwriter.addCVP(runningTime, [rightKnee.motor])

    setup_teardown_motor_combination(RightKneeExoMotor(), func, DURATION)
    state.log.download(SAVE_NAME)
    setup_teardown_motor_combination(RightKneeExoMotor(), reset_position, 6)

    state.log.plot()





