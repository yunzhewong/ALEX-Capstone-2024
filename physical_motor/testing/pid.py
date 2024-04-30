import aiosv2
import aios
import time
import math


if __name__ == "__main__":
    connected_addresses = aiosv2.get_addresses()
    connected_addresses.enable()

    connectedMotors = connected_addresses.getConnectedMotors()
    time.sleep(1)

    bottomMotor = connectedMotors[1]
    bottomMotor.setPosition(0)
    aios.getMotionCtrlConfig(bottomMotor.ip, 1)
    connected_addresses.disable()
