import math
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.TwinMotor import TwinMotor, setup_teardown_twin_motor
from dataGathering import gather_data
from aiosv2.constants import convertToMotorCommand

# 10.10.10.16 Position Gain: 50, Velocity Gain: 0.0002, Velocity Int: 0.0002, Velocity Limit: 400000 (12.566370614359172), Limit Tolerance: 1.2
# 10.10.10.17 Position Gain: 15, Velocity Gain: 0.0002, Velocity Int: 0.001, Velocity Limit: 400000 (12.566370614359172), Limit Tolerance: 1.200000048

class State:
    def __init__(self):
        self.checked = False

RUNNING_TIME = 20

def get_reference(t: float):
    remainder = t % RUNNING_TIME

    if remainder < RUNNING_TIME / 4:
        return 0
    elif remainder < RUNNING_TIME / 2:
        return math.pi / 8
    elif remainder < RUNNING_TIME * 3 / 4:
        return 0
    return math.pi / 8

if __name__ == "__main__":


    state = State()


    def func(topMotor: SafeMotor, bottomMotor: SafeMotor, runningTime: float):
        if not state.checked:
            bottomMotor.raw_motor.setPIDConfig(1, 0.0002, 0.001 , 12.5666, 1.2)
            state.checked = True

        reference = get_reference(runningTime)
        bottomMotor.setPosition(reference)

    gather_data(func, RUNNING_TIME, "Kp=1.csv")
