from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from aiosv2.TwinMotor import setup_teardown_twin_motor, TwinMotor
from aiosv2.Exoskeleton import Exoskeleton

def func(combo: Exoskeleton, runningTime: float):
    combo.leftAbductor.setCurrent(0)
    cvp = combo.leftAbductor.getCVP()
    print(cvp)

if __name__ == "__main__":
    setup_teardown_motor_combination(Exoskeleton(), func, 100)

    