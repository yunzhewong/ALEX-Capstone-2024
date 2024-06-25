import math
import time
import aiosv2.aios as aios
from aiosv2 import AiosSocket, TwinMotor



if __name__ == "__main__":
    socket = AiosSocket()
    twinMotor = TwinMotor(socket)
    twinMotor.bottomMotor.enable()
    twinMotor.topMotor.enable()

    connectedMotors = [twinMotor.bottomMotor, twinMotor.topMotor]
    for motor in connectedMotors:
        aios.reboot(motor.getIP())
        print(f"Rebooting {motor.getIP()}")

    time.sleep(10)

    for motor in connectedMotors:
        print(f"Motor {motor.getIP()} Rebooted")

    twinMotor.bottomMotor.disable()
    twinMotor.topMotor.disable()
