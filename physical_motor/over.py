from aiosv2.TwinMotor import TwinMotor, setup_teardown_twin_motor

if __name__ == "__main__":

    def func(twinMotor: TwinMotor, runningTime: float):
        if (runningTime < 5):
            twinMotor.bottomMotor.setVelocity(0.5)
        else:
            twinMotor.bottomMotor.setVelocity(-0.5)
        print(twinMotor.bottomMotor.getCVP())

    setup_teardown_twin_motor(func, 10)