import math
import time
import aiosv2.aios as aios
from aiosv2 import AiosSocket, TwinMotor


socket = AiosSocket()
twinMotor = TwinMotor(socket)
twinMotor.enable()

connectedMotors = [twinMotor.bottomMotor, twinMotor.topMotor]
for motor in connectedMotors:
    aios.reboot(motor.ip)
    print(f"Rebooting {motor.ip}")

time.sleep(10)

for motor in connectedMotors:
    aios.reboot(motor.ip)
    print(f"Motor {motor.ip} Rebooted")
    print(motor.getCVP())

twinMotor.disable()
