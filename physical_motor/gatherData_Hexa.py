from typing import Callable
from physical_motor.classes.DataLog_rightKnee import DataLog_leftKnee
from aiosv2.hexaMotor import setup_teardown_hexa_motor
from aiosv2.SafeMotorOperation import SafeMotor


def gather_data(
    command_func: Callable[[SafeMotor, SafeMotor, SafeMotor, SafeMotor, SafeMotor, SafeMotor, float], None],
    duration: float,
    name: str,
):
    dataLog = DataLog_leftKnee()

    def func(hexaMotor, runningTime: float):
        hipPitchLeft_connection = hexaMotor.hipPitchLeft
        hipPitchRight_connection = hexaMotor.hipPitchRight
        hipRollLeft_connection = hexaMotor.hipRollLeft
        hipRollRight_connection = hexaMotor.hipRollRight
        kneeLeft_connection = hexaMotor.kneeLeft
        kneeRight_connection = hexaMotor.kneeRight

        command_func(hipPitchLeft_connection, hipPitchRight_connection, hipRollLeft_connection, hipRollRight_connection, kneeLeft_connection, kneeRight_connection, runningTime)

        cvp_hipPitchLeft = hipPitchLeft_connection.getCVP()
        cvp_hipPitchRight = hipPitchRight_connection.getCVP()
        cvp_hipRollLeft = hipRollLeft_connection.getCVP()
        cvp_hipRollRigth = hipRollRight_connection.getCVP()
        cvp_kneeLeft = kneeLeft_connection.getCVP()
        cvp_kneeRight = kneeRight_connection.getCVP()

        if cvp_kneeLeft is not None:
            dataLog.addCVP(runningTime, cvp_kneeLeft)

    setup_teardown_hexa_motor(func, duration)

    dataLog.plot()
    dataLog.download(name)
