import numpy as np
from functools import reduce
from math import cos, sin


def getTraj(viaPoints, dt, Kv=0.0):

    numVia = len(viaPoints[0, :])
    numVar = len(viaPoints[:, 0])

    # Set trajectory initial conditions
    viaPointVelInit = np.zeros([numVar, 1])
    viaPointVel = np.zeros([numVar, 1])
    posTraj = np.zeros([numVia, 1])
    velTraj = np.zeros([numVia, 1])

    # Double final point to ensure processing
    viaLast = viaPoints[:, len(viaPoints[0]) - 1].copy().T
    viaLast[0] = viaLast[0] + 0.1  # Arbitrarily increase time to allow matrix inversion

    viaPoints = np.concatenate((viaPoints, viaLast.reshape(numVar, 1)), axis=1)

    for i in range(len(viaPoints[0]) - 1):
        t = viaPoints[0, i + 1] - viaPoints[0, i]

        for j in range(len(viaPoints[:, i])):
            viaPointVel[j, 0] = np.array(
                [[(viaPoints[j, i + 1] - viaPoints[j, i]) / t]]
            )

        [posSeg, velSeg] = getSegment(
            viaPoints[:, [i, i + 1]], dt, Kv * viaPointVelInit, Kv * viaPointVel
        )
        if i == 0:
            posTraj = posSeg
            velTraj = velSeg
        else:

            ## NOTE: Check axis
            posTraj = np.concatenate((posTraj, posSeg), axis=1)
            velTraj = np.concatenate((velTraj, velSeg), axis=1)

        viaPointVelInit = viaPointVel

    return [posTraj, velTraj]


def getSegment(viaPoints, dt, viaPointVelInit, viaPointVel):
    number_of_via_points = len(viaPoints[:, 0])

    coefficients = np.zeros([number_of_via_points - 1, 4])
    time_step = viaPoints[0, 1] - viaPoints[0, 0]

    for i in range(1, number_of_via_points):
        trajectory_coefficients = cubicSplineCoefficients(
            0,
            time_step,
            viaPoints[i, 0],
            viaPoints[i, 1],
            viaPointVelInit[i, 0],
            viaPointVel[i, 0],
        )
        coefficients[i - 1, :] = trajectory_coefficients.T

        # Pre-Allocate Matrix Size
    t_size = int(time_step / dt)
    posSeg = np.zeros([number_of_via_points - 1, t_size], dtype=float)
    velSeg = np.zeros([number_of_via_points - 1, t_size], dtype=float)

    j = 0
    t_tmp = 0

    while j < t_size:
        for i in range(len(coefficients[:, 0])):
            posSeg[i, j] = (
                t_tmp**3 * coefficients[i, 0]
                + t_tmp**2 * coefficients[i, 1]
                + t_tmp * coefficients[i, 2]
                + coefficients[i, 3]
            )
            velSeg[i, j] = (
                3 * t_tmp**2 * coefficients[i, 0]
                + 2 * t_tmp * coefficients[i, 1]
                + coefficients[i, 2]
            )

        j = j + 1
        t_tmp = t_tmp + dt

    return [posSeg, velSeg]


def cubicSplineCoefficients(t0, tf, x0, xf, xdot0, xdotf):
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
