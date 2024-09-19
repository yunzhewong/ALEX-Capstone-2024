
import matplotlib.pyplot as plt
import pandas as pd
import os
import math
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.misc import derivative
from rebuild import trajectoryGenerator as tg

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

SAMPLE_PERIOD = 0.04
Kp_pos = 10
Ki_pos = 0.1
Kp_vel = 10
Ki_vel = 0

trajectory_positions, trajectory_velocities = create_trajectory(SAMPLE_PERIOD)
right_knee_velocities = trajectory_velocities[5,:]
right_knee_positions = trajectory_positions[5,:]
time_array_vel = np.arange(0, len(right_knee_velocities)*SAMPLE_PERIOD, SAMPLE_PERIOD)
time_array_pos = np.arange(0, len(right_knee_positions)*SAMPLE_PERIOD, SAMPLE_PERIOD)
for i in range(len(right_knee_positions)):
    right_knee_positions[i] = math.radians(right_knee_positions[i])

for i in range(len(right_knee_velocities)):
    right_knee_velocities[i] = math.radians(right_knee_velocities[i])

file_n = f'Cascade_traj_kpp={Kp_pos}_kip={Ki_pos}_kpv={Kp_vel}_kiv={Ki_vel}'
file_p = file_n + '.csv'
df = pd.read_csv(file_p, on_bad_lines='skip')
# df = df[df['Time'] <= 61] 
# df = df.sort_values(by='Time')
# t = df['Time'].to_numpy()
# c = df[' Motor Current'].to_numpy()
# v = df[' Motor Velocities'].to_numpy()
# p = df[' Motor Positions'].to_numpy()


data = np.loadtxt(file_p, delimiter=',', skiprows=1)
t = data[:, 0]
c = data[:, 1]
v = data[:, 2]
p = data[:, 3]

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(time_array_vel, right_knee_velocities, label='reference')
ax1.plot(t, v)
ax2.plot(time_array_pos, right_knee_positions, label='reference')
ax2.plot(t, p)
plt.legend()
plt.show()
