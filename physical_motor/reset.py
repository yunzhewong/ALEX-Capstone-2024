import math
import time
import physical_motor.aiosv2.aios as aios
import aiosv2


socket = aiosv2.AiosSocket()
twinMotor = aiosv2.TwinMotor()
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
