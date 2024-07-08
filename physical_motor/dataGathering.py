from typing import Callable
from classes.DataLog import DataLog
from aiosv2.TwinMotor import setup_teardown_twin_motor
from aiosv2.SafeMotorOperation import SafeMotor, SafetyConfiguration

def gather_data(command_func: Callable[[SafeMotor, SafeMotor, float], None], duration: float, name: str):
    dataLog = DataLog()

    def func(twinMotor, runningTime: float):
        top_connection = twinMotor.topMotor
        bottom_connection = twinMotor.bottomMotor

        command_func(top_connection, bottom_connection, runningTime)

        cvp_top = top_connection.getCVP()
        cvp_bottom = bottom_connection.getCVP()

        if cvp_top is not None and cvp_bottom is not None:
            dataLog.addCVP(
                runningTime,
                cvp_top,
                cvp_bottom
            )

    setup_teardown_twin_motor(func, duration)

    dataLog.plot()
    dataLog.download(name)
