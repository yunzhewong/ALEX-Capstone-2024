from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.TwinMotor import TwinMotor
from aiosv2.RightKneeExoMotor import RightKneeExoMotor

if __name__ == "__main__":

    def func(twinMotor: RightKneeExoMotor, runningTime: float):
        if (runningTime < 5):
            twinMotor.motor.setVelocity(-0.5)
        else:
            twinMotor.motor.setVelocity(0.5)
        print(twinMotor.motor.getCVP())

    setup_teardown_motor_combination(RightKneeExoMotor(), func, 10)