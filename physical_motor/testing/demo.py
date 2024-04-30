import aiosv2
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


if __name__ == "__main__":
    connected_addresses = aiosv2.get_addresses()
    connected_addresses.enable()

    connectedMotors = connected_addresses.getConnectedMotors()
    time.sleep(1)

    bottomMotor = connectedMotors[1]
    positionControl(bottomMotor)
    velocityControl(bottomMotor)
    currentControl(bottomMotor)

    connected_addresses.disable()
