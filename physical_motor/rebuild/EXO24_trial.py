from typing import List
import aios
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import trajectoryGenerator as tg


NO_ROBOT = True


def deg_to_aios(deg_val):
    return deg_val / 3.0


class ExoJoint:
    def __init__(self, id, name, ip, viaPoints):
        self.id = id
        self.name = name
        self.ip = ip
        self.viaPoints = viaPoints


def create_joints() -> List[ExoJoint]:
    joints = []
    joints.append(ExoJoint(0, "L-Abd", "10.10.10.29", []))
    joints.append(ExoJoint(1, "R-Abd", "10.10.10.39", []))
    joints.append(ExoJoint(2, "L-Ext", "10.10.10.10", []))
    joints.append(ExoJoint(3, "R-Ext", "10.10.10.36", []))
    joints.append(ExoJoint(4, "L-Knee", "10.10.10.8", []))
    joints.append(ExoJoint(5, "R-Knee", "10.10.10.30", []))
    return joints


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


def plot_trajectory(
    trajectory_positions, trajectory_velocities, joints: List[ExoJoint], dt
):
    tt = np.arange(0, len(trajectory_positions[0, :]) * dt, dt)
    fig, ax = plt.subplots(2, sharex=True)
    for j in range(0, len(trajectory_positions[:, 0])):
        ax[0].plot(tt, trajectory_positions[j, :], label=joints[j].name)
        ax[1].plot(tt, trajectory_velocities[j, :], label=joints[j].name)
    plt.legend(loc="upper right")
    plt.show(block=True)


def connect_robot(joints: List[ExoJoint]):
    Server_IP_list = aios.broadcast_func()

    # Check that expected motors are connected
    if Server_IP_list:
        for joint in joints:
            if joint.ip not in Server_IP_list:
                print("NOT ALL MOTORS CONNECTED!\n")
                return False
    else:
        print("NO MOTORS CONNECTED!\n")
        return False

    # Check that all encoders are ready
    for joint in joints:
        if not aios.encoderIsReady(joint.ip, 1):
            print("ENCODER NOT READY!\n")
            return False

    # Get motor attributes
    for joint in joints:
        aios.getRoot(joint.ip)

    # Enable motor drives
    for joint in joints:
        enableSuccess = aios.enable(joint.ip, 1)
        if not enableSuccess:
            print("MOTOR FAILED TO ENABLE\n")
            return False
    return True


def main():
    dt = 0.02

    joints = create_joints()
    trajectory_positions, trajectory_velocities = create_trajectory(dt)

    if NO_ROBOT:
        plot_trajectory(trajectory_positions, trajectory_velocities, joints, dt)
    else:
        connected_successfully = connect_robot(joints)
        if not connected_successfully:
            raise Exception("Failed to connect to robot")

    # Ready Message
    print("EXO22 is ready. Commencing trajectory...\n")
    time.sleep(0.5)

    # Run Trajectory
    VELOCITY_FEEDFORWARD = 0.0
    TORQUE_FEEDFORWARD = 1.0

    init_time = time.time()
    for n in range(len(trajectory_positions[0, :])):

        # Start Loop Timer
        start = time.time()
        running_time = start - init_time

        # Send Position to Motors
        if NO_ROBOT:
            print(f"[EXO22 NO_ROBOT] t={running_time:.4} \t", end="")

        # for joint in joints:
        #     joint_position = trajectory_positions[joint.id, n]

        #     if NO_ROBOT:
        #         print(f"{joint_position:.4}\t", end="")
        #     else:
        #         aios.setInputPosition_pt(
        #             joint.ip,
        #             deg_to_aios(joint_position),
        #             VELOCITY_FEEDFORWARD,
        #             TORQUE_FEEDFORWARD,
        #         )

        for joint in joints:
            joint_velocity = trajectory_velocities[joint.id, n]

            if NO_ROBOT:
                print(f"{joint_velocity:.4}\t", end="")
            else:
                aios.setInputVelocity(
                    joint.ip,
                    deg_to_aios(joint_velocity),
                    TORQUE_FEEDFORWARD,
                )


        if NO_ROBOT:
            print("")

        # Delay Loop
        time.sleep(0.01)

        # Measure Loop Length
        end = time.time()

        # Print Loop Time
        if end - start > 0.05:
            print("WARNING: Long loop time: ", end - start)


if __name__ == "__main__":
    main()
