from aiosv2 import AiosSocket, TwinMotor
import aiosv2.aios as aios
import time


if __name__ == "__main__":
    socket = AiosSocket()
    twinMotor = TwinMotor(socket)
    twinMotor.bottomMotor.enable()
    twinMotor.topMotor.enable()

    time.sleep(1)

    topPID = twinMotor.topMotor.raw_motor.requestPIDConfig()
    bottomPID = twinMotor.bottomMotor.raw_motor.requestPIDConfig()
    print("Top Motor:", topPID)
    print("Bottom Motor:", bottomPID)
    twinMotor.bottomMotor.disable()
    twinMotor.topMotor.disable()
