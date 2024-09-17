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
import matplotlib.pyplot as plt


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

def get_value(time_array, value_array, running_time):
    index = np.argmin(np.abs(time_array - running_time))
    return value_array[index]



if __name__ == "__main__":
    dt = 0.04
    trajectory_positions, trajectory_velocities = create_trajectory(dt)
    right_knee_traj = trajectory_velocities[5, :]
    t = np.arange(0, len(right_knee_traj)*dt, dt)

    rt_arr = np.arange(0, 60, 0.1)
    set_traj_arr = []
    for running_time in rt_arr:
        set_traj_arr.append(get_value(t, right_knee_traj, running_time))

    
    # Plot time vs right knee trajectory
    plt.plot(t, right_knee_traj)
    plt.plot(rt_arr, set_traj_arr, label='set_value')
    plt.xlabel('Time (s)')
    plt.ylabel('Right Knee Trajectory')
    plt.title('Right Knee Trajectory over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

