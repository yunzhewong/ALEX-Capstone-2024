import aiosv2
from aiosv2.TwinMotor import setup_teardown_twin_motor
import time
import math


def positionControl(motor: aiosv2.ConnectedMotor):
    WAITING_TIME_S = 0.4
    motor.setPosition(0)
    time.sleep(WAITING_TIME_S)
    motor.setPosition(math.pi / 8)
    time.sleep(WAITING_TIME_S)
    motor.setPosition(0)
    time.sleep(WAITING_TIME_S)
    motor.setPosition(-math.pi / 8)
    time.sleep(WAITING_TIME_S)
    motor.setPosition(0)
    time.sleep(WAITING_TIME_S)


def velocityControl(motor: aiosv2.ConnectedMotor):
    WAITING_TIME_S = 1
    motor.setVelocity(0)
    time.sleep(WAITING_TIME_S)
    motor.setVelocity(math.pi / 8)
    time.sleep(WAITING_TIME_S)
    motor.setVelocity(0)
    time.sleep(WAITING_TIME_S)
    motor.setVelocity(-math.pi / 8)
    time.sleep(WAITING_TIME_S)
    motor.setVelocity(0)
    time.sleep(WAITING_TIME_S)


def currentControl(motor: aiosv2.ConnectedMotor):
    WAITING_TIME_S = 1
    motor.setCurrent(0)
    time.sleep(WAITING_TIME_S)
    motor.setCurrent(-1)
    time.sleep(WAITING_TIME_S)
    motor.setCurrent(0)
    time.sleep(WAITING_TIME_S)
    motor.setCurrent(1)
    time.sleep(WAITING_TIME_S)
    motor.setCurrent(0)
    time.sleep(WAITING_TIME_S)


def func(twinMotor: aiosv2.TwinMotor):
    bottomMotor = twinMotor.bottomMotor
    positionControl(bottomMotor)
    velocityControl(bottomMotor)
    currentControl(bottomMotor)


if __name__ == "__main__":
    setup_teardown_twin_motor(func)
    