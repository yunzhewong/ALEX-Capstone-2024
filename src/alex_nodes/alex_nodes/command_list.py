from enum import Enum
import math
import trajectoryGenerator as tg
import numpy as np
from commands import CommandType


def stable_exo(state, t: float):
    state.types = [
        CommandType.Position.value,
        CommandType.Position.value,
        CommandType.Position.value,
        CommandType.Position.value,
        CommandType.Position.value,
        CommandType.Position.value,
    ]
    state.values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class State(Enum):
    Collecting = 1
    Paused = 2
    Resetting = 3


def step_analysis(state, t: float):
    MAX_ANGLE = 3 * math.pi
    MAX_TIME = 10
    STARTING_ANGLE = -3 * math.pi
    RESET_TIME = 5
    PAUSE_TIME = 1
    START_CURRENT = 0.2
    INCREMENT = 0.02

    if state.state == State.Collecting:
        command = START_CURRENT + INCREMENT * state.collect_index
        if not state.state_initialised:
            state.collecting_start = t
            state.state_initialised = True
            state.log.open(f"step{format(round(command, 2), '.2f')}A.csv")
            print(command)

        state.log.write(t, command, state.velocities[1], state.positions[1])

        exceeded_max_angle = state.positions[1] >= MAX_ANGLE
        exceeded_max_time = (t - state.collecting_start) >= MAX_TIME

        should_stop = exceeded_max_angle or exceeded_max_time

        if should_stop:
            state.state = State.Paused
            state.post_pause_state = State.Resetting
            state.collect_index += 1
            state.log.close()
            state.state_initialised = False

        state.types = [CommandType.Position.value, CommandType.Current.value]
        state.values = [0.0, command]
    elif state.state == State.Paused:
        if not state.state_initialised:
            state.pause_end_time = t + PAUSE_TIME
            state.state_initialised = True

        if t > state.pause_end_time:
            state.state = state.post_pause_state
            state.state_initialised = False

        state.types = [CommandType.Position.value, CommandType.Current.value]
        state.values = [0.0, 0.0]
    elif state.state == State.Resetting:
        if not state.state_initialised:
            state.reset_start = t
            state.reset_end = t + RESET_TIME
            state.reset_angle = state.positions[1]
            state.state_initialised = True

        expected_pos = state.reset_angle - (
            (state.reset_angle - STARTING_ANGLE) / RESET_TIME
        ) * (t - state.reset_start)

        if t > state.reset_end:
            state.post_pause_state = State.Collecting
            state.state = State.Paused
            state.state_initialised = False

        state.types = [CommandType.Position.value, CommandType.Position.value]
        state.values = [0.0, float(expected_pos)]


def position_test(state, t: float):
    remainder = t % 10

    state.types = [CommandType.Position.value, CommandType.Position.value]
    if remainder < 5:
        state.values = [0.0, math.pi]
    else:
        state.values = [math.pi, 0.0]


def exo_demo_trajectory(dt):
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
    Knee_R = [
        10.0,
        15.0,
        22.0,
        25.0,
        22.0,
        15.0,
        10.0,
        7.0,
        5.0,
        7.0,
        10.0,
        20.0,
        30.0,
        45.0,
        55.0,
        60.0,
        55.0,
        40.0,
        25.0,
        15.0,
        10.0,
    ]
    assert len(Knee_R) == GAIT_CYCLE_POSITIONS_LENGTH

    HipExt_R = [
        -35.0,
        -34.0,
        -32.0,
        -30.0,
        -25.0,
        -17.0,
        -10.0,
        -5.0,
        -0.0,
        5.0,
        8.0,
        10.0,
        5.0,
        0.0,
        -12.0,
        -25.0,
        -30.0,
        -33.0,
        -35.0,
        -37.0,
        -35.0,
    ]
    assert len(HipExt_R) == GAIT_CYCLE_POSITIONS_LENGTH

    HipAbd_R = [
        0.0,
        1.0,
        4.0,
        7.0,
        10.0,
        7.0,
        4.0,
        2.0,
        1.0,
        0.0,
        0.0,
        -1.0,
        -2.0,
        -2.0,
        -2.0,
        -1.0,
        -0.0,
        0.0,
        0.0,
        1.0,
        1.0,
    ]
    assert len(HipAbd_R) == GAIT_CYCLE_POSITIONS_LENGTH

    OFFSET_LENGTH = math.floor(GAIT_CYCLE_POSITIONS_LENGTH / 2)

    HipAbd_L = -np.roll(HipAbd_R, OFFSET_LENGTH)
    HipExt_L = -np.roll(HipExt_R, OFFSET_LENGTH)
    Knee_L = -np.roll(Knee_R, OFFSET_LENGTH)

    ## Gait Cycles
    GAIT_CYCLES = 2
    for _ in range(GAIT_CYCLES):
        for i in range(0, GAIT_CYCLE_POSITIONS_LENGTH - 1):
            viaPoints.append(
                np.array(
                    [
                        get_time(init_time, i * 0.05),
                        -6 + HipAbd_L[i],
                        6 + HipAbd_R[i],
                        HipExt_L[i],
                        HipExt_R[i],
                        Knee_L[i],
                        Knee_R[i],
                    ]
                )
            )
        init_time = init_time + GAIT_CYCLE_TIME

    # Side Step Trajectory
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

    # Generate Trajectory
    vp = np.zeros([len(viaPoints), len(viaPoints[0])])
    for i in range(len(viaPoints)):
        vp[i] = viaPoints[i]
    return tg.getTrajectory(vp.T, dt, Kv=0.75)


def follow_demo_trajectory(state, t: float):

    index = math.floor(t / state.dt)

    if index >= len(state.trajectories[0, :]):
        state.types = [CommandType.Position.value for _ in range(6)]
        state.values = [0.0 for _ in range(6)]
        return

    array = state.trajectories[:, index]

    state.types = [CommandType.Position.value for _ in range(6)]
    state.values = [float(array[i]) for i in range(6)]
