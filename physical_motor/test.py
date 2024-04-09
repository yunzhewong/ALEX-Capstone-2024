import time
import aios
import aiosv2

connected_addresses = aiosv2.get_addresses()
print("INITALISATION")
connected_addresses.enable()
motor_ips = connected_addresses.get_motor_ips()

time.sleep(2)

print("MOVEMENT")
for motor_ip in motor_ips:
    aios.controlMode(aios.ControlMode.POSITION_CONTROL.value, motor_ip, 1)
    aios.setPosition(1, 0, 0, True, motor_ip, 1)
print()

time.sleep(3)

connected_addresses.disable()
