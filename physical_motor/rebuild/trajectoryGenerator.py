import numpy as np


def getTrajectory(via_points, time_delta, Kv=0.0):
    number_of_joints = len(via_points[:, 0])
    number_of_via_points = len(via_points[0, :])

    # Set trajectory initial conditions
    trajectory_positions = np.zeros([number_of_via_points, 1])
    trajector_velocities = np.zeros([number_of_via_points, 1])
    via_point_initial_velocity = np.zeros([number_of_joints, 1])
    via_point_final_velocity = np.zeros([number_of_joints, 1])

    # Double final point to ensure processing
    final_via_point_copy = via_points[:, len(via_points[0]) - 1].copy()
    final_via_point_copy[0] = (
        final_via_point_copy[0] + 0.1
    )  # Arbitrarily increase time to allow matrix inversion

    via_points = np.concatenate(
        (via_points, final_via_point_copy.reshape(number_of_joints, 1)), axis=1
    )

    for i in range(number_of_via_points):
        t = via_points[0, i + 1] - via_points[0, i]

        for j in range(len(via_points[:, i])):
            via_point_final_velocity[j, 0] = Kv * np.array(
                [[(via_points[j, i + 1] - via_points[j, i]) / t]]
            )

        segment_positions, segment_velocities = getSegment(
            via_points[:, [i, i + 1]],
            time_delta,
            via_point_initial_velocity,
            via_point_final_velocity,
        )
        if i == 0:
            trajectory_positions = segment_positions
            trajector_velocities = segment_velocities
        else:
            ## NOTE: Check axis
            trajectory_positions = np.concatenate(
                (trajectory_positions, segment_positions), axis=1
            )
            trajector_velocities = np.concatenate(
                (trajector_velocities, segment_velocities), axis=1
            )

        via_point_initial_velocity = via_point_final_velocity

    return trajectory_positions, trajector_velocities


def getSegment(viaPoints, time_delta, viaPointVelInit, viaPointVel):
    number_of_joints = len(viaPoints[:, 0])

    coefficients = np.zeros([number_of_joints - 1, 4])
    time_between_points = viaPoints[0, 1] - viaPoints[0, 0]

    for i in range(1, number_of_joints):
        trajectory_coefficients = findCubicSplineCoefficients(
            0,
            time_between_points,
            viaPoints[i, 0],
            viaPoints[i, 1],
            viaPointVelInit[i, 0],
            viaPointVel[i, 0],
        )
        coefficients[i - 1, :] = trajectory_coefficients.T

        # Pre-Allocate Matrix Size
    total_points = int(time_between_points / time_delta)
    positions = np.zeros([number_of_joints - 1, total_points], dtype=float)
    velocities = np.zeros([number_of_joints - 1, total_points], dtype=float)

    for j in range(total_points):
        t = j * time_delta
        for i in range(len(coefficients[:, 0])):
            positions[i, j], velocities[i, j] = evaluteCubicSpline(
                coefficients[i, :], t
            )

    return (positions, velocities)


def findCubicSplineCoefficients(t0, tf, x0, xf, xdot0, xdotf):
    # Ax = b , x = A^(-1)b
    A = np.array(
        [
            [t0**3, t0**2, t0, 1],
            [3 * t0**2, 2 * t0, 1, 0],
            [tf**3, tf**2, tf, 1],
            [3 * tf**2, 2 * tf, 1, 0],
        ]
    )
    b = np.array([[x0], [xdot0], [xf], [xdotf]])

    return np.dot(np.linalg.inv(A), b)


def evaluteCubicSpline(coefficients, time):
    position = (
        time**3 * coefficients[0]
        + time**2 * coefficients[1]
        + time * coefficients[2]
        + coefficients[3]
    )
    velocity = (
        3 * time**2 * coefficients[0] + 2 * time * coefficients[1] + coefficients[2]
    )
    return position, velocity
