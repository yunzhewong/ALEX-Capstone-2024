from aiosv2.TwinMotor import setup_teardown_twin_motor, TwinMotor



def func(twinmotor: TwinMotor, runningTime: float):
    cvp = twinmotor.bottomMotor.getCVP()

    if cvp is None:
        return
    
    twinmotor.bottomMotor.setCurrent(0)
    print(cvp)

if __name__ == "__main__":
    setup_teardown_twin_motor(func, 15)
    