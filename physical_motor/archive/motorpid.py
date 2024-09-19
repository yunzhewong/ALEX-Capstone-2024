import math
from aiosv2.TwinMotor import TwinMotor, setup_teardown_twin_motor
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.ConnectedMotor import ConnectedMotor
from aiosv2.AiosSocket import AiosSocket
from dataGathering import gather_data
from aiosv2.constants import ExoskeletonMotorConverter


# 10.10.10.16 Position Gain: 50, Velocity Gain: 0.0002, Velocity Int: 0.0002, Velocity Limit: 400000 (12.566370614359172), Limit Tolerance: 1.2
# 10.10.10.17 Position Gain: 15, Velocity Gain: 0.0002, Velocity Int: 0.001, Velocity Limit: 400000 (12.566370614359172), Limit Tolerance: 1.200000048

# 10.10.10.29 PGain = 10, VGain = 2, VInt=0.04, vel_limit=40
# 10.10.10.30 PGain = 10, VGain = 2, VInt=0.04, vel_limit=40
# 10.10.10.39 PGain = 60, VGain = 2, VInt=0.04, vel_limit=40
# 10.10.10.8 PGain = 60, VGain = 2, VInt=0.04, vel_limit=40
# 10.10.10.36 PGain = 60, VGain = 2, VInt=0.04, vel_limit=40
# 10.10.10.10 PGain = 60, VGain = 2, VInt=0.04, vel_limit=40

if __name__ == "__main__":
    socket = AiosSocket()
    raw_motor = ConnectedMotor("10.10.10.39", socket, ExoskeletonMotorConverter())
    raw_motor.requestPIDConfig()

    json_obj, ip = socket.readJSON()
    print(json_obj)



