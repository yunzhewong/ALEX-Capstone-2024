from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from aiosv2.TwinMotor import setup_teardown_twin_motor, TwinMotor


def func(twinMotor: TwinMotor, runningTime: float):
    cvp = twinMotor.bottomMotor.getCVP()
    twinMotor.bottomMotor.setCurrent(0)
    print(cvp)

if __name__ == "__main__":
    setup_teardown_twin_motor(func, 100)

    