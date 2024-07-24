import math
from aiosv2.TwinMotor import TwinMotor, setup_teardown_twin_motor
from aiosv2.constants import convertToMotorCommand

# 10.10.10.16 Position Gain: 50, Velocity Gain: 0.0002, Velocity Int: 0.0002, Velocity Limit: 400000 (12.566370614359172), Limit Tolerance: 1.2

class State:
    def __init__(self):
        self.checked = False


if __name__ == "__main__":


    state = State()

    def reboot(twinMotor: TwinMotor, runningTime: float):
        if not state.checked:
            twinMotor.topMotor.raw_motor.requestRestart()
            state.checked = True


    def setPID(twinMotor: TwinMotor, runningTime: float):
        if not state.checked:
            twinMotor.topMotor.raw_motor.setPIDConfig(50, 0.0002, 0.0002, 1, 1.4)
            state.checked = True
        
    def checkPID(twinMotor: TwinMotor, runningTime: float):
        if not state.checked:
            twinMotor.topMotor.raw_motor.requestPIDConfig()
            state.checked = True


    def func(twinMotor: TwinMotor, runningTime: float):

        twinMotor.topMotor.setVelocity(-1 * runningTime)
        print(twinMotor.topMotor.getCVP())

    setup_teardown_twin_motor(func, 5)
