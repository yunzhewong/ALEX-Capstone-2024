import numpy as np
import matplotlib.pyplot as plt

# Example joint names, replace these with the actual joint objects.
class Joint:
    def __init__(self, name):
        self.name = name

# Example joints, replace with actual joints from EXO24.py
joints = [
    Joint("leftAbductor"),
    Joint("rightAbductor"),
    Joint("leftExtensor"),
    Joint("rightExtensor"),
    Joint("leftKnee"),
    Joint("rightKnee")
]

def get_time(init_time, offset):
    """ Calculate the time for each trajectory point based on the initial time and an offset. """
    return init_time + offset

def interpolate_trajectory(viaPoints, dt):
    """ Interpolates trajectory points to consider velocity control. """
    times = [point[0] for point in viaPoints]
    trajectories = [point[1:] for point in viaPoints]

    # Calculate the time steps
    time_steps = np.arange(times[0], times[-1], dt)

    interpolated_trajectories = np.zeros((len(time_steps), len(trajectories[0])))

    # Interpolating positions
    for i in range(len(trajectories[0])):
        positions = [trajectory[i] for trajectory in trajectories]
        interpolated_trajectories[:, i] = np.interp(time_steps, times, positions)

    return time_steps, interpolated_trajectories

def create_trajectory(dt, speed_scale=1.0):
    viaPoints = []
    init_time = 0.0  # Initialize time

    # WIDEN HIPS
    viaPoints.append(np.array([0.0, -0.0, 0.0, 0.0, -0.0, -0.0, 0.0]))
    viaPoints.append(np.array([1.0, -6.0, 6.0, 0.0, -0.0, -0.0, 0.0]))

    # SIT TO STAND
    viaPoints.append(np.array([4.0, -6.0, 6.0, 45.0, -45.0, -90.0, 90.0]))
    viaPoints.append(np.array([5.0, -6.0, 6.0, 45.0, -45.0, -90.0, 90.0]))
    viaPoints.append(np.array([8.0, -6.0, 6.0, 0.0, -0.0, -0.0, 0.0]))

    # Side Step Trajectory
    SIDE_STEP_CYCLES = 2
    GAIT_CYCLE_TIME = 1.0  # Example time for a gait cycle
    for _ in range(SIDE_STEP_CYCLES):
        viaPoints.append(
            np.array([get_time(init_time, 0.05), -6.0, 6.0, 10.0, -10.0, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.15), -6.0, 6.0, 10.0, -30.0, -10.0, 65.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.30), -0.0, 30.0, 10.0, -30.0, -10.0, 65.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.50), -0.0, 30.0, 10.0, -10.0, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.65), -30.0, 0.0, 10.0, -10.0, -10.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.80), -30.0, 0.0, 30.0, -10.0, -65.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 0.90), -6.0, 6.0, 30.0, -10.0, -65.0, 10.0])
        )
        viaPoints.append(
            np.array([get_time(init_time, 1.0), -6.0, 6.0, 10.0, -10.0, -10.0, 10.0])
        )
        init_time = init_time + GAIT_CYCLE_TIME

    # RESET HIPS
    viaPoints.append(
        np.array([get_time(init_time, 0.10), -6.0, 6.0, 0.0, 0.0, -0.0, 0.0])
    )
    viaPoints.append(
        np.array([get_time(init_time, 0.20), -0.0, 0.0, 0.0, -0.0, -0.0, 0.0])
    )

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

def plot_trajectory(trajectory_positions, trajectory_velocities, joints, dt):
    tt = np.arange(0, len(trajectory_positions[0, :]) * dt, dt)
    fig, ax = plt.subplots(2, sharex=True)
    for j in range(len(trajectory_positions[:, 0])):
        ax[0].plot(tt, trajectory_positions[j, :], label=joints[j].name)
        ax[1].plot(tt, trajectory_velocities[j, :], label=joints[j].name)
    plt.legend(loc="upper right")
    plt.show()

# Define time step
dt = 0.01

# Generate the trajectory
trajectory_positions, trajectory_velocities = create_trajectory(dt)

# Plot the trajectory
plot_trajectory(trajectory_positions, trajectory_velocities, joints, dt)
