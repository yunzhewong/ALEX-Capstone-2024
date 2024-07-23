from TwinMotor import setup_teardown_twin_motor
import aiosv2


if __name__ == "__main__":

    checked = False

    def func(twinMotor: aiosv2.TwinMotor, runningTime: float):
        if not checked:
            twinMotor.topMotor.raw_motor.requestPIDConfig()
            checked = True
        twinMotor.topMotor.setVelocity(4)

    setup_teardown_twin_motor(func, 15)
