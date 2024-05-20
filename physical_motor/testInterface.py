import aiosv2
import time
import math
import numpy as np

if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    twinMotor.bottomMotor.setPosition(0)
    time.sleep(1)
    twinMotor.bottomMotor.setPosition(math.pi / 8)
    time.sleep(1)
    twinMotor.bottomMotor.setPosition(0)
    time.sleep(1)
    twinMotor.bottomMotor.setPosition(-math.pi / 8)
    time.sleep(1)
    twinMotor.bottomMotor.setPosition(0)
    time.sleep(1)

    twinMotor.disable()
