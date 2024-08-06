
import numpy as np

def findCubicSpline(t0, tf, x0, xf, xdot0, xdotf):
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

    column_vector = np.dot(np.linalg.inv(A), b) 

    return [float(column_vector[i]) for i in range(4)]


def evaluateCubicSpline(coefficients, time):

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
