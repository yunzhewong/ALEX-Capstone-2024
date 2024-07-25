from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor



def func(exoMotor: RightKneeExoMotor, runningTime: float):

    cvp = exoMotor.motor.getCVP()

    if cvp is None:
        return
    
    exoMotor.motor.setCurrent(0)
    print(cvp)

if __name__ == "__main__":
    setup_teardown_rightknee_exomotor(func, 15)

    