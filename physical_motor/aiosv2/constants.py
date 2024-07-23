from enum import Enum
import math


PORT_rt = 2333  # Real-time control data port, ie. speed, position, current and other real-time data
PORT_srv = 2334  # Low priority service data port. ie, parameter setting and reading
PORT_pt = 10000  # Passthrough port


# Actuator control mode
class ControlMode(Enum):
    Voltage = 0
    Current = 1
    Velocity = 2
    Position = 3
    Trajectory = 4


# Motor State (Active / Inactive)
class AxisState(Enum):
    AXIS_STATE_UNDEFINED = 0
    AXIS_STATE_IDLE = 1
    AXIS_STATE_STARTUP_SEQUENCE = 2
    AXIS_STATE_MOTOR_CALIBRATION = 4
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
    AXIS_STATE_ENABLE = 8


CONVERSION_RATIO = 100000 / math.pi


def convertToMotorCommand(value):
    return value * CONVERSION_RATIO


def convertFromMotorCommand(value):
    return value / CONVERSION_RATIO
