import aiosv2
from aiosv2.TwinMotor import setup_teardown_twin_motor
import time
import math

PAUSE_TIME = 1

def positionControl(motor: aiosv2.SafeMotor, currentTime: float):
    change = math.pi / 8
    positions = [0, change, 0, -1 * change, 0]

    index = math.floor(currentTime)
    if index >= len(positions):
        return
    
    motor.setPosition(positions[index])



def velocityControl(motor: aiosv2.SafeMotor, currentTime: float):
    change = math.pi / 8
    velocities = [0, change, 0, -1 * change, 0]

    index = math.floor(currentTime)
    if index >= len(velocities):
        return

    motor.setVelocity(velocities[index])


def currentControl(motor: aiosv2.SafeMotor, currentTime: float):
    change = 1

    currents = [0, change, 0, -1 * change, 0]

    index = math.floor(currentTime)
    if index >= len(currents):
        return

    motor.setCurrent(currents[index])

def func(twinMotor: aiosv2.TwinMotor, currentTime: float):
    bottomMotor = twinMotor.bottomMotor
    if (currentTime < 5):
        positionControl(bottomMotor, currentTime)
    elif currentTime < 10:
        velocityControl(bottomMotor, currentTime - 5)
    elif currentTime < 15:
        currentControl(bottomMotor, currentTime - 10)

if __name__ == "__main__":
    setup_teardown_twin_motor(func, 15)
    