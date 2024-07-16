from aiosv2.TwinMotor import setup_teardown_twin_motor, TwinMotor
from aiosv2 import aios



def func(twinmotor: TwinMotor, runningTime: float):
    cvp = twinmotor.bottomMotor.getCVP()

    if cvp is None:
        return
    
    aios.getAbsEncoder(twinmotor.bottomMotor.getIP())

if __name__ == "__main__":
    setup_teardown_twin_motor(func, 15)
    