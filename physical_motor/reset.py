import math
import time
import aios
import aiosv2

connected_addresses = aiosv2.get_addresses()
connected_addresses.enable()

connectedMotors = connected_addresses.getConnectedMotors()

motor = connectedMotors[1]

aios.reboot(motor.ip)
time.sleep(10)
print(motor.getCVP())

connected_addresses.disable()
