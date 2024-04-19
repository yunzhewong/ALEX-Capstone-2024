import time
import aios
import aiosv2

connected_addresses = aiosv2.get_addresses()

print("INITALISATION")
connected_addresses.enable()
motor_ips = connected_addresses.get_motor_ips()

time.sleep(2)

print("MOVEMENT")

PRIMARY_MOTOR = motor_ips[0]
# for motor_ip in motor_ips:
#     aios.controlMode(aios.ControlMode.CURRENT_CONTROL.value, motor_ip, 1)
#     aios.setCurrent(-1, True, motor_ip, 1)
# aios.setInputPosition_pt(motor_ips[0], 0.1, 0, 0)

aios.controlMode(aios.ControlMode.CURRENT_CONTROL.value, PRIMARY_MOTOR, 1)
aios.getCVP(PRIMARY_MOTOR, 1)
aios.setCurrent(1, False, PRIMARY_MOTOR, 1)
for i in range(1, 100):
    aios.getCVP(PRIMARY_MOTOR, 1)
    time.sleep(0.01)
aios.setCurrent(0, False, PRIMARY_MOTOR, 1)
aios.getCVP(PRIMARY_MOTOR, 1)
print()

time.sleep(1)

connected_addresses.disable()
