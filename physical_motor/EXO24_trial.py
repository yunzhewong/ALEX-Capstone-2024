import numpy as np
import time
import matplotlib.pyplot as plt
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.ControlLoop import setup_teardown_motor_combination

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

def create_trajectory(dt, speed_scale=1.0):
    viaPoints = []

    # WIDEN HIPS
    viaPoints.append(np.array([0.0, -0.0, 0.0, 0.0, -0.0, -0.0, 0.0]))
    viaPoints.append(np.array([1.0, -6.0, 6.0, 0.0, -0.0, -0.0, 0.0]))

    # SIT TO STAND
    viaPoints.append(np.array([4.0, -6.0, 6.0, 45.0, -45.0, -90.0, 90.0]))
    viaPoints.append(np.array([5.0, -6.0, 6.0, 45.0, -45.0, -90.0, 90.0]))
    viaPoints.append(np.array([8.0, -6.0, 6.0, 0.0, -0.0, -0.0, 0.0]))

    # Time interpolation
    times = [point[0] for point in viaPoints]
    total_time = times[-1]
    n_steps = int(total_time / dt) + 1

    trajectory_positions = np.zeros((len(viaPoints[0]) - 1, n_steps))
    trajectory_velocities = np.zeros((len(viaPoints[0]) - 1, n_steps))

    for i in range(1, len(viaPoints[0])):
        positions = [point[i] for point in viaPoints]
        trajectory_positions[i - 1, :] = np.interp(
            np.linspace(0, total_time, n_steps), times, positions
        )

    trajectory_velocities = np.gradient(trajectory_positions, axis=1) / dt
    trajectory_velocities *= speed_scale  # Adjust the speed by scaling velocities

    return trajectory_positions, trajectory_velocities


# Function to plot the trajectory if NO_ROBOT is True
def plot_trajectory(trajectory_positions, trajectory_velocities, joints, dt):
    tt = np.arange(0, len(trajectory_positions[0, :]) * dt, dt)
    fig, ax = plt.subplots(2, sharex=True)
    for j in range(len(trajectory_positions[:, 0])):
        ax[0].plot(tt, trajectory_positions[j, :], label=joints[j].name)
        ax[1].plot(tt, trajectory_velocities[j, :], label=joints[j].name)
    plt.legend(loc="upper right")
    plt.show()

# Function to move the exoskeleton along the trajectory using velocity control
def move_to_trajectory(exoskeleton: Exoskeleton, runningTime: float):
    dt = 0.02

    # Create joints
    joints = create_joints(exoskeleton)

    # Generate trajectory
    trajectory_positions, trajectory_velocities = create_trajectory(dt)

    # If NO_ROBOT is True, plot the trajectory
    if NO_ROBOT:
        plot_trajectory(trajectory_positions, trajectory_velocities, joints, dt)
    else:
        exoskeleton.enable()
        init_time = time.time()
        for n in range(len(trajectory_velocities[0, :])):
            start = time.time()
            running_time = start - init_time

            # Send Velocity to Motors
            for joint, vel in zip(joints, trajectory_velocities[:, n]):
                joint.setVelocity(vel)  # Use the setVelocity method to control the motors by velocity

            time.sleep(0.01)  # Delay Loop

            end = time.time()
            if end - start > 0.05:
                print("WARNING: Long loop time: ", end - start)

        exoskeleton.disable()

if __name__ == "__main__":
    setup_teardown_motor_combination(Exoskeleton(no_robot=NO_ROBOT), move_to_trajectory, 10, no_robot=NO_ROBOT)
