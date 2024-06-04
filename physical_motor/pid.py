from aiosv2 import AiosSocket, TwinMotor
import aiosv2.aios as aios
import time


if __name__ == "__main__":
    socket = AiosSocket()
    twinMotor = TwinMotor(socket)
    twinMotor.enable()

    time.sleep(1)

    topPID = twinMotor.topMotor.getPIDConfig()
    bottomPID = twinMotor.bottomMotor.getPIDConfig()
    print("Top Motor:", topPID)
    print("Bottom Motor:", bottomPID)
    twinMotor.disable()
