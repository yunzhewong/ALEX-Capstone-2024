# the frequency that gazebo runs the simulation at
import math


SIMULATION_FREQUENCY = 1000
SIMULATION_PERIOD = 1 / SIMULATION_FREQUENCY

MOTOR_NETWORKING_FREQUENCY = 300
MOTOR_NETWORKING_PERIOD = 1 / MOTOR_NETWORKING_FREQUENCY

VELOCITY_ERROR_MULTIPLIER = (180 / 3) / math.pi
DAMPING_SLOPE = -13.83
DAMPING_INTERCEPT = 51.0314
MOTOR_TORQUE_CONSTANT = 0.124 * 120
KINETIC_FRICTION = 8.1003
STATIC_FRICTION = 10.4160
EPSILON = 0.005