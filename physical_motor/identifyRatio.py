import math
import time
import aiosv2

connected_addresses = aiosv2.get_addresses()
connected_addresses.enable()

connectedMotors = connected_addresses.getConnectedMotors()

motor = connectedMotors[1]

FINAL_POSITION = math.pi / 2
STEPS = 5
for i in range(STEPS + 1):
    position = i * FINAL_POSITION / STEPS
    motor.setPosition(position)
    time.sleep(0.5)

for i in range(STEPS + 1):
    position = FINAL_POSITION - i * FINAL_POSITION / STEPS
    motor.setPosition(position)
    time.sleep(0.5)

connected_addresses.disable()
