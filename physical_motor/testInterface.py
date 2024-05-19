import aiosv2
import time

if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    motor = twinMotor.bottomMotor
    motor.setPosition(0)
    time.sleep(1)
    motor.setPosition(5)
    time.sleep(1)
    motor.setPosition(0)
    time.sleep(1)
    motor.setPosition(-5)
    time.sleep(1)
    motor.setPosition(0)

    twinMotor.disable()
