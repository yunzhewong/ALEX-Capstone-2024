import aiosv2
import time

if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    motor = aiosv2.ConnectedMotor("10.10.10.16", socket)
    for _ in range(100):
        motor.setPosition(0)
        time.sleep(0.01)
        motor.setPosition(5)
        time.sleep(0.01)
        motor.setPosition(0)
        time.sleep(0.01)
        motor.setPosition(-5)
        time.sleep(0.01)
        motor.setPosition(0)
