from enum import Enum
import math
import utils.trajectoryGenerator as tg
import numpy as np
from utils.commands import CommandType


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


def create_trajectory(dt):
    viaPoints = []

    # Time, L-Abd, R-Abd, L-Ext, R-Ext, L-Knee, R-Knee
    # WIDEN HIPS
    viaPoints.append(np.array([0.0, 6.0, -6.0, 0.0, -0.0, -0.0, 0.0]))
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
    Ext_R = [
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
    Abd_R = [
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
                    [
                        get_gait_time(init_time, i * 0.05),
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
            np.array(
                [
                    get_side_step_cycle_time(init_time, 0.05),
                    -0.0,
                    0.0,
                    10.0,
                    -10.0,
                    -10.0,
                    10.0,
                ]
            )
        )
        viaPoints.append(
            np.array(
                [
                    get_side_step_cycle_time(init_time, 0.15),
                    -0.0,
                    0.0,
                    10.0,
                    -30.0,
                    -10.0,
                    65.0,
                ]
            )
        )
        viaPoints.append(
            np.array(
                [
                    get_side_step_cycle_time(init_time, 0.30),
                    6,
                    24,
                    10.0,
                    -30.0,
                    -10.0,
                    65.0,
                ]
            )
        )
        viaPoints.append(
            np.array(
                [
                    get_side_step_cycle_time(init_time, 0.50),
                    6,
                    24,
                    10.0,
                    -10,
                    -10.0,
                    10.0,
                ]
            )
        )
        viaPoints.append(
            np.array(
                [
                    get_side_step_cycle_time(init_time, 0.65),
                    -24,
                    -6,
                    10.0,
                    -10,
                    -10.0,
                    10.0,
                ]
            )
        )
        viaPoints.append(
            np.array(
                [
                    get_side_step_cycle_time(init_time, 0.80),
                    -24,
                    -6,
                    30.0,
                    -10.0,
                    -65.0,
                    10.0,
                ]
            )
        )
        viaPoints.append(
            np.array(
                [
                    get_side_step_cycle_time(init_time, 0.90),
                    -0,
                    0,
                    30.0,
                    -10.0,
                    -65.0,
                    10.0,
                ]
            )
        )
        viaPoints.append(
            np.array(
                [
                    get_side_step_cycle_time(init_time, 1),
                    -0,
                    0,
                    10.0,
                    -10.0,
                    -10.0,
                    10.0,
                ]
            )
        )
        init_time = init_time + SIDE_STEP_CYCLE_TIME

    # RESET HIPS
    viaPoints.append(np.array([init_time + 1, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0]))
    viaPoints.append(np.array([init_time + 2, 6.0, -6.0, 0.0, -0.0, -0.0, 0.0]))
    init_time = init_time + 2

    # Convert via points to trajectory
    vp = np.zeros([len(viaPoints), len(viaPoints[0])])
    for i in range(len(viaPoints)):
        vp[i] = viaPoints[i]

    return tg.getTrajectory(vp.T, dt, Kv=0.75)


SAMPLE_PERIOD = 1 / 300


class Exo24Trajectory:
    def __init__(self):
        positions, velocities = create_trajectory(SAMPLE_PERIOD)
        self.positions = positions / 180 * math.pi
        self.velocities = velocities / 180 * math.pi

    def get_state(self, t):
        index = math.floor(t / SAMPLE_PERIOD)
        position = self.positions[:, index]
        velocity = self.velocities[:, index]

        return position, velocity
