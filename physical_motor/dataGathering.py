from typing import Callable
from classes.DataLog import DataLog
from aiosv2.TwinMotor import setup_teardown_twin_motor
from aiosv2.SafeMotorOperation import SafeMotor


def gather_data(command_func, duration, save_name, desired_trajectory_bottom=None, desired_trajectory_top=None, desired_velocity_bottom=None, desired_velocity_top=None):
    dataLog = DataLog(
        desired_trajectory_bottom=desired_trajectory_bottom,
        desired_trajectory_top=desired_trajectory_top,
        desired_velocity_bottom=desired_velocity_bottom,
        desired_velocity_top=desired_velocity_top
    )

    def func(twinMotor, runningTime: float):
        top_connection = twinMotor.topMotor
        bottom_connection = twinMotor.bottomMotor

        command_func(top_connection, bottom_connection, runningTime)

        cvp_top = top_connection.getCVP()
        cvp_bottom = bottom_connection.getCVP()

        if cvp_top is not None and cvp_bottom is not None:
            dataLog.addCVP(runningTime, cvp_top, cvp_bottom)

    setup_teardown_twin_motor(func, duration)

    dataLog.download(save_name)
    dataLog.plot()
