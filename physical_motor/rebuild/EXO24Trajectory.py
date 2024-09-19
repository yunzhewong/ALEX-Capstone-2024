import math
import numpy as np
import rebuild.trajectoryGenerator as tg




def create_trajectory(dt):
    viaPoints = []
    
    # Time, L-Abd, R-Abd, L-Ext, R-Ext, L-Knee, R-Knee
    # WIDEN HIPS
    viaPoints.append(np.array([0.0, 2.0, -2.0, 0.0, -0.0, -0.0, 0.0]))
    viaPoints.append(np.array([1.0, -0.0, 0.0, 0.0, -0.0, -0.0, 0.0]))
    
    # SIT TO STAND
    viaPoints.append(np.array([5.0, -0.0, 0.0, 45.0, -45.0, -90.0, 90.0]))
    viaPoints.append(np.array([6.0, -0.0, 0.0, 45.0, -45.0, -90.0, 90.0]))
    viaPoints.append(np.array([10.0, -0.0, 0.0, 0.0, -0.0, -0.0, 0.0]))

    init_time = 10 + 2

    GAIT_CYCLE_TIME = 12.0
    def get_gait_time(start_time: float, gait_percentage: float):
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
                    [   get_gait_time(init_time, i * 0.05),
                        Abd_L[i],
                        Abd_R[i],
                        Ext_L[i],
                        Ext_R[i],
                        Knee_L[i],
                        Knee_R[i],
                    ]
                )
            )
        init_time = init_time + GAIT_CYCLE_TIME

    SIDE_STEP_CYCLE_TIME = 24.0
    def get_side_step_cycle_time(start_time: float, gait_percentage: float):
        return start_time + gait_percentage * SIDE_STEP_CYCLE_TIME


    SIDE_STEP_CYCLES = 2
    for _ in range(SIDE_STEP_CYCLES):
        viaPoints.append(
            np.array([get_side_step_cycle_time(init_time, 0.05), -0.0, 0.0, 10.0, -10.0, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_side_step_cycle_time(init_time, 0.15), -0.0, 0.0, 10.0, -30.0, -10.0, 65.0])
        )
        viaPoints.append(
            np.array([get_side_step_cycle_time(init_time, 0.30), 6, 24, 10.0, -30.0, -10.0, 65.0])
        )
        viaPoints.append(
            np.array([get_side_step_cycle_time(init_time, 0.50), 6, 24, 10.0, -10, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_side_step_cycle_time(init_time, 0.65), -24, -6, 10.0, -10, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_side_step_cycle_time(init_time, 0.80), -24, -6, 30.0, -10.0, -65.0, 10.0])
        )
        viaPoints.append(
            np.array([get_side_step_cycle_time(init_time, 0.90), -0, 0, 30.0, -10.0, -65.0, 10.0])
        )
        viaPoints.append(
            np.array([get_side_step_cycle_time(init_time, 1), -0, 0, 10.0, -10.0, -10.0, 10.0])
        )
        init_time = init_time + SIDE_STEP_CYCLE_TIME

    # RESET HIPS
    viaPoints.append(
        np.array([init_time + 1, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0])
    )
    viaPoints.append(
        np.array([init_time + 2, 2.0, -2.0, 0.0, -0.0, -0.0, 0.0])
    )
    init_time = init_time + 2


    # Convert via points to trajectory
    vp = np.zeros([len(viaPoints), len(viaPoints[0])])
    for i in range(len(viaPoints)):
        vp[i] = viaPoints[i]
    
    return tg.getTrajectory(vp.T, dt, Kv=0.75)

class Exo24Trajectory():
    def __init__(self, dt):
        positions, velocities = create_trajectory(dt)
        self.dt = dt
        self.positions = positions / 180 * math.pi
        self.velocities = velocities / 180 * math.pi

    def get_state(self, t):
        index = math.floor(t / self.dt)
        position = self.positions[:, index]
        velocity = self.velocities[:, index]

        return position, velocity