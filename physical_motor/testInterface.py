import aiosv2
import time
import math

if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    time.sleep(2)
    motor = twinMotor.bottomMotor
    motor.setPosition(0)
    time.sleep(1)
    motor.setPosition(math.pi / 8)
    time.sleep(1)
    motor.setPosition(0)
    time.sleep(1)
    motor.setPosition(-math.pi / 8)
    time.sleep(1)
    motor.setPosition(0)
    time.sleep(1)

    twinMotor.disable()
