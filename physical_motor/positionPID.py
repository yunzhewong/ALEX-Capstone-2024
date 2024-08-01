import math
from aiosv2.TwinMotor import TwinMotor, setup_teardown_twin_motor
from aiosv2.SafeMotorOperation import SafeMotor
from dataGathering import gather_data

# 10.10.10.16 Position Gain: 50, Velocity Gain: 0.0002, Velocity Int: 0.0002, Velocity Limit: 400000 (12.566370614359172), Limit Tolerance: 1.2
# 10.10.10.17 Position Gain: 15, Velocity Gain: 0.0002, Velocity Int: 0.001, Velocity Limit: 400000 (12.566370614359172), Limit Tolerance: 1.200000048

class State():
    def __init__(self):
        self.checked = False

if __name__ == "__main__":


    state = State()


    def func(twinMotor: TwinMotor, runningTime: float):
        if not state.checked:
            twinMotor.bottomMotor.raw_motor.requestPIDConfig()
            twinMotor.topMotor.raw_motor.requestPIDConfig()
            state.checked = True


    setup_teardown_twin_motor(func, 3)
