from typing import Callable
from aiosv2.Calibration import CalibrationState
from aiosv2.AiosSocket import AiosSocket
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.DataStream import DataStream
from aiosv2.constants import ExoskeletonMotorConverter
from aiosv2.readConfig import readConfigurationJSON, destructureMotorCombinationConfig, removePositionLimits
from aiosv2.ControlLoop import MotorCombination, setup_teardown_motor_combination

class RightKneeExoMotor(MotorCombination):
    def __init__(self, overrideConfiguration: dict | None = None):
        socket = AiosSocket()

        motorConfiguration = readConfigurationJSON(["config", "RightKneeExoMotor.json"])
        if overrideConfiguration is not None:
            motorConfiguration = overrideConfiguration

        expected_ips, motors = destructureMotorCombinationConfig(motorConfiguration)        
        socket.assertConnectedAddresses(expected_ips)

        self.calibrationData = readConfigurationJSON(['config', 'TwinMotorCalibration.json'])
        calibrationAdjustments = self.calibrationData['adjustments']

        motorConverter = ExoskeletonMotorConverter()

        self.motor = SafeMotor(motors[0], socket, motorConverter, calibrationAdjustments[0])

        self.dataStream = DataStream(socket, [self.motor], motorConverter)

    def enable(self):
        self.motor.enable()
        self.dataStream.enable()

    def requestReadyCheck(self):
        self.motor.requestReadyCheck()
        
    def isReady(self):
        return self.motor.isReady()
    
    def logCalibrationData(self):
        print()
        print(f"Calibration was last done: {self.calibrationData['date']}")
        print(f"Motor Position: {self.motor.getCVP().position:.4f}")
        print()        

    def getStreamError(self):
        return self.dataStream.errored()

    def disable(self):
        self.motor.disable()
        self.dataStream.disable()


def setup_teardown_rightknee_exomotor(actions: Callable[[RightKneeExoMotor, float], None], totalRunningTime: float):
    setup_teardown_motor_combination(RightKneeExoMotor(), actions, totalRunningTime)

def calibrate_right_knee_exomotor():
    calibrationConfiguration = readConfigurationJSON(["config", "RightKneeCalibrationSetup.json"])
    state = CalibrationState(calibrationConfiguration)

    def func(exoMotor: RightKneeExoMotor, _: float):
        if state.motors is None:
            state.motors = [exoMotor.motor]
        
        state.iterate()

    motorConfiguration = readConfigurationJSON(["config", "RightKneeExoMotor.json"])
    removePositionLimits(motorConfiguration)

    setup_teardown_motor_combination(RightKneeExoMotor(motorConfiguration), func, 120)