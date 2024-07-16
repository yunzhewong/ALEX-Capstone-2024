import aios_alter as aios
import time
import threading
import numpy as np
import math
import matplotlib.pyplot as plt
import json
import Trajectory_Generator as tg


def deg_to_aios(deg_val):
    return deg_val / 3.0


Server_IP_list = []
viaPoints = []
joints = []

dt = 0.02
positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
feedback = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
encoderIsReady = True
NO_ROBOT = True

# Add Actuators
joints_names = ["L-Abd", "R-Abd", "L-Ext", "R-Ext", "L-Knee", "R-Knee"]
joints.append({"ID": 0, "IP": "10.10.10.29", "viaPoints": []})  # L-Abd
joints.append({"ID": 1, "IP": "10.10.10.39", "viaPoints": []})  # R-Abd
joints.append({"ID": 2, "IP": "10.10.10.10", "viaPoints": []})  # L-Ext
joints.append({"ID": 3, "IP": "10.10.10.36", "viaPoints": []})  # R-Ext
joints.append({"ID": 4, "IP": "10.10.10.8", "viaPoints": []})  # L-Knee
joints.append({"ID": 5, "IP": "10.10.10.30", "viaPoints": []})  # R-Knee

# Add Via Points
# Time, L-Abd, R-Abd, L-Ext, R-Ext, L-Knee, R-Knee

# WIDEN HIPS
viaPoints.append(np.array([0.0, -0.0, 0.0, 0.0, -0.0, -0.0, 0.0]))
viaPoints.append(np.array([1.0, -6.0, 6.0, 0.0, -0.0, -0.0, 0.0]))

# SIT TO STAND
viaPoints.append(np.array([4.0, -6.0, 6.0, 45.0, -45.0, -90.0, 90.0]))
viaPoints.append(np.array([5.0, -6.0, 6.0, 45.0, -45.0, -90.0, 90.0]))
viaPoints.append(np.array([8.0, -6.0, 6.0, 0.0, -0.0, -0.0, 0.0]))

# WALKING: LEFT STEP
# viaPoints.append(np.array([0.0, -2.0, 2.0, 0.0, -0.0, -0.0, 0.0]))
# viaPoints.append(np.array([2.0, -2.0, 2.0, 0.0, -0.0, -0.0, 0.0]))
# viaPoints.append(np.array([2.0, -2.0, 2.0, 5.0, -0.0, -10.0, 0.0]))
# viaPoints.append(np.array([3.0, -2.0, 2.0, 10.0, 8.0, -5.0, 0.0]))
# viaPoints.append(np.array([3.8, -2.0, 2.0, 5.0, 8.0, -3.0, 8.0]))
# viaPoints.append(np.array([5.0, -2.0, 2.0, 5.0, -0.0, -3.0, 20.0]))
# viaPoints.append(np.array([6.2, -2.0, 2.0, -8.0, -10.0, -0.0, 5.0]))
# viaPoints.append(np.array([8.0, -2.0, 2.0, -8.0, -5.0, -8.0, 3.0]))


# L-Ext (+ve Flex, -ve Ext)
# R-Ext (-ve Flex, +ve Ext)
# L-Knee (-ve Flex, +ve Ext)
# R-Knee (+ve Flex, -ve Ext)
#                           Time,               L-Abd,R-Abd,L-Ext,R-Ext,L-Knee,R-Knee
init_time = 1 + 1 + 8
gait_cycle_t = 12.0

# Two Gait Cycles
# viaPoints.append(np.array([init_time+5/100*gait_cycle_t, -6, 6, 25.0, -35.0, -65.0, 10.0]))
# viaPoints.append(np.array([init_time+15/100*gait_cycle_t, -6, 6, 35.0, -30.0, -30.0, 30.0]))
# viaPoints.append(np.array([init_time+40/100*gait_cycle_t, -6, 6, 20.0, 0.0, -10.0, 5.0]))
# viaPoints.append(np.array([init_time+55/100*gait_cycle_t, -6, 6, 35.0, 5.0, -12.0, 20.0]))
# viaPoints.append(np.array([init_time+75/100*gait_cycle_t, -6, 6, 20.0, -25.0, -30.0, 65.0]))
# viaPoints.append(np.array([init_time+85/100*gait_cycle_t, -6, 6, 0.0, -35.0, -10.0, 30.0]))
# viaPoints.append(np.array([init_time+100/100*gait_cycle_t, -6, 6, 5.0, -20.0, -20.0, 10.0]))

# init_time=init_time+gait_cycle_t;
# viaPoints.append(np.array([init_time+5/100*gait_cycle_t, -6, 6, 25.0, -35.0, -65.0, 10.0]))
# viaPoints.append(np.array([init_time+15/100*gait_cycle_t, -6, 6, 35.0, -30.0, -30.0, 30.0]))
# viaPoints.append(np.array([init_time+40/100*gait_cycle_t, -6, 6, 20.0, 0.0, -10.0, 5.0]))
# viaPoints.append(np.array([init_time+55/100*gait_cycle_t, -6, 6, 35.0, 5.0, -12.0, 20.0]))
# viaPoints.append(np.array([init_time+75/100*gait_cycle_t, -6, 6, 20.0, -25.0, -30.0, 65.0]))
# viaPoints.append(np.array([init_time+85/100*gait_cycle_t, -6, 6, 0.0, -35.0, -10.0, 30.0]))
# viaPoints.append(np.array([init_time+100/100*gait_cycle_t, -6, 6, 5.0, -20.0, -20.0, 10.0]))


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

HipAbd_L = -np.roll(HipAbd_R, math.floor(len(HipAbd_R) / 2))
HipExt_L = -np.roll(HipExt_R, math.floor(len(HipExt_R) / 2))
Knee_L = -np.roll(Knee_R, math.floor(len(Knee_R) / 2))

## Gait Cycles
for j in range(2):
    for i in range(0, len(Knee_R) - 1):
        viaPoints.append(
            np.array(
                [
                    init_time + i * 5 / 100 * gait_cycle_t,
                    -6 + HipAbd_L[i],
                    6 + HipAbd_R[i],
                    HipExt_L[i],
                    HipExt_R[i],
                    Knee_L[i],
                    Knee_R[i],
                ]
            )
        )
    init_time = init_time + gait_cycle_t


# Side Step Trajectory

for j in range(2):
    viaPoints.append(
        np.array(
            [init_time + 5 / 100 * gait_cycle_t, -6.0, 6.0, 10.0, -10.0, -10.0, 10.0]
        )
    )
    viaPoints.append(
        np.array([init_time + 15 / 100 * gait_cycle_t, -6, 6, 10.0, -30.0, -10.0, 65.0])
    )
    viaPoints.append(
        np.array(
            [init_time + 30 / 100 * gait_cycle_t, -0, 30, 10.0, -30.0, -10.0, 65.0]
        )
    )
    viaPoints.append(
        np.array([init_time + 50 / 100 * gait_cycle_t, -0, 30, 10.0, -10, -10.0, 10.0])
    )
    viaPoints.append(
        np.array([init_time + 65 / 100 * gait_cycle_t, -30, 0, 10.0, -10, -10.0, 10.0])
    )
    viaPoints.append(
        np.array(
            [init_time + 80 / 100 * gait_cycle_t, -30, 0, 30.0, -10.0, -65.0, 10.0]
        )
    )
    viaPoints.append(
        np.array([init_time + 90 / 100 * gait_cycle_t, -6, 6, 30.0, -10.0, -65.0, 10.0])
    )
    viaPoints.append(
        np.array(
            [init_time + 100 / 100 * gait_cycle_t, -6, 6, 10.0, -10.0, -10.0, 10.0]
        )
    )
    init_time = init_time + gait_cycle_t


# viaPoints.append(np.array([2.0, -2.0, 2.0, 6.0, 3.0, -0.0, 0.0]))
# viaPoints.append(np.array([3.6, -2.0, 2.0, 5.0, 3.5, -1.5, 6.0]))
# viaPoints.append(np.array([4.4, -2.0, 2.0, 3.0, -6.0, -3.0, 17.0]))
# viaPoints.append(np.array([6.0, -2.0, 2.0,-3.0, -6.0, -0.0, 0.0]))
# viaPoints.append(np.array([7.6, -2.0, 2.0, -3.5, -5.0, -6.0, 1.5]))
# viaPoints.append(np.array([8.4, -2.0, 2.0, 6.0, -3.0, -17.0, 3.0]))
# viaPoints.append(np.array([10.0, -2.0, 2.0,6.0, 3.0, -0.0, 0.0]))


# RESET HIPS
viaPoints.append(
    np.array([init_time + 10 / 100 * gait_cycle_t, -6.0, 6.0, 0.0, 0.0, -0.0, 0.0])
)
viaPoints.append(
    np.array([init_time + 20 / 100 * gait_cycle_t, -0.0, 0.0, 0.0, -0.0, -0.0, 0.0])
)


# Generate Trajectory
vp = np.zeros([len(viaPoints), len(viaPoints[0])])
for i in range(len(viaPoints)):
    vp[i] = viaPoints[i]
[posTraj, velTraj] = tg.getTraj(vp.T, dt, Kv=0.75)


if NO_ROBOT:
    tt = np.arange(0, len(posTraj[0, :]) * dt, dt)
    fig, ax = plt.subplots(2, sharex=True)
    for j in range(0, len(posTraj[:, 0])):
        ax[0].plot(tt, posTraj[j, :], label=joints_names[j])
        ax[1].plot(tt, velTraj[j, :], label=joints_names[j])
    plt.legend(loc="upper right")
    plt.show(block=True)


def main():

    if not NO_ROBOT:

        # Discover connected motors
        Server_IP_list = aios.broadcast_func()

        # Check that expected motors are connected
        if Server_IP_list:
            for i in range(len(joints)):
                if joints[i]["IP"] not in Server_IP_list:
                    print("NOT ALL MOTORS CONNECTED!\n")
                    return
        else:
            print("NO MOTORS CONNECTED!\n")
            return

        # Check that all encoders are ready
        for i in range(len(joints)):
            if not aios.encoderIsReady(joints[i]["IP"], 1):
                encoderIsReady = False
                print("ENCODER NOT READY!\n")
                return

        # Get motor attributes
        for i in range(len(joints)):
            aios.getRoot(joints[i]["IP"])

        # Enable motor drives
        for i in range(len(joints)):
            enableSuccess = aios.enable(joints[i]["IP"], 1)
            if not enableSuccess:
                print("MOTOR FAILED TO ENABLE\n")
                return

    # Ready Message
    print("EXO22 is ready. Commencing trajectory...\n")
    time.sleep(0.5)

    # Run Trajectory
    velocity_ff = 0.0
    torque_ff = 1.0

    init_time = time.time()
    for n in range(len(posTraj[0, :])):

        # Start Loop Timer
        start = time.time()

        # Send Position to Motors
        if NO_ROBOT:
            print("[EXO22 NO_ROBOT] t=", f"{start-init_time:.4}", "\t", end="")
        for i in range(len(joints)):
            if NO_ROBOT:
                #    print("[EXO22 NO_ROBOT] ", "ID:", joints[i]['ID'], ", Pos: ", posTraj[joints[i]['ID'], n])
                print(f"{posTraj[joints[i]['ID'], n]:.4}", "\t", end="")
            else:
                # aios.setInputPosition_pt(joints[i]['IP'], deg_to_aios(posTraj[joints[i]['ID'], n]), deg_to_aios(velTraj[joints[i]['ID'], n]), torque_ff)
                aios.setInputPosition_pt(
                    joints[i]["IP"],
                    deg_to_aios(posTraj[joints[i]["ID"], n]),
                    velocity_ff,
                    torque_ff,
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
