import time
from typing import Callable
from aiosv2.Calibration import CalibrationState
from aiosv2 import AiosSocket
from aiosv2.constants import TwinMotorConverter, logPosition
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.DataStream import DataStream
from aiosv2.readConfig import readConfigurationJSON, removePositionLimits, destructureMotorCombinationConfig
from aiosv2.ControlLoop import MotorCombination, setup_teardown_motor_combination


class TwinMotor(MotorCombination):
    def __init__(self, overrideConfiguration:dict | None = None):
        configuration = readConfigurationJSON(["config", "TwinMotor.json"])
        if overrideConfiguration is not None:
            configuration = overrideConfiguration

        socket = AiosSocket()
        expected_ips, motors = destructureMotorCombinationConfig(configuration)        
        socket.assertConnectedAddresses(expected_ips)
        motorConverter = TwinMotorConverter()

        self.calibrationData = readConfigurationJSON(['config', 'TwinMotorCalibration.json'])
        calibrationAdjustments = self.calibrationData['adjustments']

        self.topMotor = SafeMotor(motors[0], socket, motorConverter, calibrationAdjustments[0])
        self.bottomMotor = SafeMotor(motors[1], socket, motorConverter, calibrationAdjustments[1])
        self.dataStream = DataStream(
            socket, [self.topMotor, self.bottomMotor], motorConverter
        )

    def enable(self):
        self.topMotor.enable()  # Enable the top motor
        self.bottomMotor.enable()  # Enable the bottom motor
        self.dataStream.enable()

    def requestReadyCheck(self):
        self.bottomMotor.requestReadyCheck(1)
        self.topMotor.requestReadyCheck(1)

    def isReady(self):
        return self.topMotor.isReady() and self.bottomMotor.isReady()

    def logCalibrationData(self):
        print()
        print(f"Calibration was last done: {self.calibrationData['date']}")
        print(f"Top Motor Position: {logPosition(self.topMotor.getCVP().position)}")
        print(f"Bottom Motor Position: {logPosition(self.bottomMotor.getCVP().position)}")
        print()        

    def getStreamError(self):
        return self.dataStream.errored()

    def disable(self):
        self.topMotor.disable()  # Disable the top motor
        self.bottomMotor.disable()  # Disable the bottom motor
        self.dataStream.disable()

    def flush(self):
        self.bottomMotor.flush()
        self.topMotor.flush()


def calibrate_twin_motor():
    calibrationConfiguration = readConfigurationJSON(["config", "TwinMotorCalibrationSetup.json"])
    state = CalibrationState(calibrationConfiguration, ["config", "TwinMotorCalibration.json"], [0, None])

    def func(twinMotor: TwinMotor, _: float):
        if state.motors is None:
            state.motors = [twinMotor.topMotor, twinMotor.bottomMotor]
        
        state.iterate()

    motorConfiguration = readConfigurationJSON(["config", "TwinMotor.json"])
    removePositionLimits(motorConfiguration)

    setup_teardown_motor_combination(TwinMotor(motorConfiguration), func, 120)

def setup_teardown_twin_motor(actions: Callable[[TwinMotor, float], None], totalRunningTime: float):
    setup_teardown_motor_combination(TwinMotor(), actions, totalRunningTime)
    