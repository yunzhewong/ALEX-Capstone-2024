from typing import List
import aiosv2
from aiosv2.TwinMotor import setup_teardown_twin_motor
import time
import math

def getValue(offset: float, values: List[float]):
    index = math.floor(offset)
    if index >= len(values):
        return None
    return values[index]


def positionControl(motor: aiosv2.SafeMotor, offset: float):
    change = math.pi / 8
    position = getValue(offset, [0, change, 0, -1 * change, 0])

    if position is not None:
        motor.setPosition(position)

def velocityControl(motor: aiosv2.SafeMotor, offset: float):
    change = math.pi / 8
    velocity = getValue(offset, [0, change, 0, -1 * change, 0])

    if velocity is not None:
        motor.setVelocity(velocity)

def currentControl(motor: aiosv2.SafeMotor, offset: float):
    change = 1
    current = getValue(offset, [0, change, 0, -1 * change, 0])

    if current is not None:
        motor.setCurrent(current)

def func(twinMotor: aiosv2.TwinMotor, runningTime: float):
    bottomMotor = twinMotor.bottomMotor
    if (runningTime < 5):
        positionControl(bottomMotor, runningTime)
    elif runningTime < 10:
        velocityControl(bottomMotor, runningTime - 5)
    elif runningTime < 15:
        currentControl(bottomMotor, runningTime - 10)

if __name__ == "__main__":
    setup_teardown_twin_motor(func, 15)
    