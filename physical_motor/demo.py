import aiosv2
from aiosv2.TwinMotor import setup_teardown_twin_motor
import time
import math

PAUSE_TIME = 1

def positionControl(motor: aiosv2.SafeMotor):
    change = math.pi / 8
    positions = [0, change, 0, -1 * change, 0]

    for position in positions:
        motor.setPosition(position)
        time.sleep(PAUSE_TIME)


def velocityControl(motor: aiosv2.SafeMotor):
    change = math.pi / 8
    velocities = [0, change, 0, -1 * change, 0]

    for velocity in velocities:
        motor.setVelocity(velocity)
        time.sleep(PAUSE_TIME)


def currentControl(motor: aiosv2.SafeMotor):
    change = 1

    currents = [0, change, 0, -1 * change, 0]

    for current in currents:
        motor.setCurrent(current)
        time.sleep(PAUSE_TIME)

def func(twinMotor: aiosv2.TwinMotor):
    bottomMotor = twinMotor.bottomMotor
    positionControl(bottomMotor)
    velocityControl(bottomMotor)
    currentControl(bottomMotor)


if __name__ == "__main__":
    setup_teardown_twin_motor(func)
    