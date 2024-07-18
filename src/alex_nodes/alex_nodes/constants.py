# the frequency that gazebo runs the simulation at
SIMULATION_FREQUENCY = 1000
SIMULATION_PERIOD = 1 / SIMULATION_FREQUENCY

# the frequency that the motor operates at (used for reading/publishing commands)
SEND_FREQUENCY = 300
SEND_PERIOD = 1 / SEND_FREQUENCY
MOTOR_TORQUE_CONSTANT = 0.1 # K_t

# obtained from the motor
POSITION_GAIN = 15
VEL_GAIN = 0.0002
VEL_INTEGRATOR_GAIN = 0.001

EPSILON = 0.001
FRICTION_ADJUSTMENT = 0.01 * 0