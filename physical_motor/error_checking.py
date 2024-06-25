import time
from aiosv2.ConnectedMotor import ConnectedMotor
from aiosv2.AiosSocket import AiosSocket
from aiosv2 import aios

if __name__ == "__main__":
    socket = AiosSocket()
    motor = ConnectedMotor(ip="10.10.10.17", socket=socket)

    motor.enable()

    motor.setCurrent(10)
    time.sleep(0.5)
    motor.setCurrent(0)

    aios.getError(motor.ip, 1)
    motor.disable()