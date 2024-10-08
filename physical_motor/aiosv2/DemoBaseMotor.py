import time
from typing import Callable
from aiosv2.Calibration import CalibrationState
from aiosv2 import AiosSocket
from aiosv2.constants import TwinMotorConverter, logPosition
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.DataStream import DataStream
from aiosv2.readConfig import readConfigurationJSON, removePositionLimits, destructureMotorCombinationConfig
from aiosv2.ControlLoop import MotorCombination, setup_teardown_motor_combination


class DemoBaseMotor(MotorCombination):
    def __init__(self, overrideConfiguration:dict | None = None):
        configuration = readConfigurationJSON(["config", "DemoBaseMotor.json"])
        if overrideConfiguration is not None:
            configuration = overrideConfiguration

        socket = AiosSocket()
        expected_ips, motors = destructureMotorCombinationConfig(configuration)        
        socket.assertConnectedAddresses(expected_ips)
        motorConverter = TwinMotorConverter()

        self.calibrationData = readConfigurationJSON(['config', 'DemoBaseMotorCalibration.json'])
        calibrationAdjustments = self.calibrationData['adjustments']

        self.motor = SafeMotor(motors[0], socket, motorConverter, calibrationAdjustments[0])
        self.dataStream = DataStream(
            socket, [self.motor], motorConverter
        )

    def enable(self):
        self.motor.enable() 
        self.dataStream.enable()

    def requestReadyCheck(self):
        self.motor.requestReadyCheck(1)

    def isReady(self):
        return self.motor.isReady()

    def logCalibrationData(self):
        print()
        print(f"Calibration was last done: {self.calibrationData['date']}")
        print(f"Motor Position: {logPosition(self.motor.getCVP().position)}")
        print()        

    def getStreamError(self):
        return self.dataStream.errored()

    def disable(self):
        self.motor.disable() 
        self.dataStream.disable()

    def flush(self):
        self.motor.flush()

def calibrate_demo_base_motor():
    calibrationConfiguration = readConfigurationJSON(["config", "DemoBaseMotorCalibrationSetup.json"])
    state = CalibrationState(calibrationConfiguration, ["config", "DemoBaseMotorCalibration.json"], [None])

    def func(demoBaseMotor: DemoBaseMotor, _: float):
        if state.motors is None:
            state.motors = [demoBaseMotor.motor]
        
        state.iterate()

    motorConfiguration = readConfigurationJSON(["config", "DemoBaseMotor.json"])
    removePositionLimits(motorConfiguration)

    setup_teardown_motor_combination(DemoBaseMotor(motorConfiguration), func, 120)
