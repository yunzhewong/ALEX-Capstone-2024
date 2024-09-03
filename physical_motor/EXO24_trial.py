import trajectory
import numpy as np
import time
import matplotlib.pyplot as plt
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.ControlLoop import setup_teardown_motor_combination

# Set the NO_ROBOT flag as needed
NO_ROBOT = True

# Function to create joints similar to EXO24.py
def create_joints(exoskeleton: Exoskeleton):
    return [
        exoskeleton.leftAbductor,
        exoskeleton.rightAbductor,
        exoskeleton.leftExtensor,
        exoskeleton.rightExtensor,
        exoskeleton.leftKnee,
        exoskeleton.rightKnee
    ]

# Function to plot the trajectory if NO_ROBOT is True
def plot_trajectory(trajectory_positions, trajectory_velocities, joints, dt):
    tt = np.arange(0, len(trajectory_positions[0, :]) * dt, dt)
    fig, ax = plt.subplots(2, sharex=True)
    for j in range(len(trajectory_positions[:, 0])):
        ax[0].plot(tt, trajectory_positions[j, :], label=joints[j].name)
        ax[1].plot(tt, trajectory_velocities[j, :], label=joints[j].name)
    plt.legend(loc="upper right")
    plt.show()

# Function to move the exoskeleton along the trajectory
def move_to_trajectory(exoskeleton: Exoskeleton, runningTime: float):
    dt = 0.02

    # Create joints
    joints = create_joints(exoskeleton)

    # Define the trajectory points (simplified example)
    trajectory_positions = np.array([
        [0.0, -0.0, 0.0, 0.0, -0.0, -0.0],
        [1.0, -6.0, 6.0, 0.0, -0.0, -0.0],
        [4.0, -6.0, 6.0, 45.0, -45.0, -90.0],
        [5.0, -6.0, 6.0, 45.0, -45.0, -90.0],
        [8.0, -6.0, 6.0, 0.0, -0.0, -0.0]
    ])
    trajectory_velocities = np.gradient(trajectory_positions, axis=0) / dt

    # If NO_ROBOT is True, plot the trajectory
    if NO_ROBOT:
        plot_trajectory(trajectory_positions, trajectory_velocities, joints, dt)
    else:
        exoskeleton.enable()
        init_time = time.time()
        for n in range(len(trajectory_positions[:, 0])):
            start = time.time()
            running_time = start - init_time

            # Send Position to Motors
            for joint, pos in zip(joints, trajectory_positions[:, n]):
                trajectory.slow_move_to_pos(joint, pos)

            time.sleep(0.01)  # Delay Loop

            end = time.time()
            if end - start > 0.05:
                print("WARNING: Long loop time: ", end - start)

        exoskeleton.disable()


if __name__ == "__main__":
    setup_teardown_motor_combination(Exoskeleton(no_robot=NO_ROBOT), move_to_trajectory, 10, no_robot=NO_ROBOT)
