import aiosv2
import physical_motor.aiosv2.aios as aios
import time
import math


if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    time.sleep(1)

    twinMotor.bottomMotor.setPosition(0)
    aios.getMotionCtrlConfig(twinMotor.bottomMotor.ip, 1)
    twinMotor.disable()
