import math
import time
import aios
import aiosv2


socket = aiosv2.AiosSocket()
connected_addresses = socket.get_addresses()
connected_addresses.enable()

connectedMotors = connected_addresses.getConnectedMotors()

for motor in connectedMotors:
    aios.reboot(motor.ip)
    print(f"Rebooting {motor.ip}")

time.sleep(10)

for motor in connectedMotors:
    aios.reboot(motor.ip)
    print(f"Motor {motor.ip} Rebooted")
    print(motor.getCVP())

connected_addresses.disable()
