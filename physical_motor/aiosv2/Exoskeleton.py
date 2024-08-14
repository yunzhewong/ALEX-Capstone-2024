import math
import time
from typing import Callable
from aiosv2.Calibration import CalibrationState
from aiosv2.AiosSocket import AiosSocket
from aiosv2.constants import ExoskeletonMotorConverter, TwinMotorConverter
from aiosv2.SafeMotorOperation import SafeMotor, SafetyConfiguration
from aiosv2.DataStream import DataStream
from aiosv2.readConfig import readConfigurationJSON, destructureMotorCombinationConfig, removePositionLimits
from aiosv2.ControlLoop import MotorCombination, setup_teardown_motor_combination


class Exoskeleton(MotorCombination):
    def __init__(self, overrideConfiguration: dict | None = None):
        socket = AiosSocket()

        motorConfiguration = readConfigurationJSON(["config", "Exoskeleton.json"])
        if overrideConfiguration is not None:
            motorConfiguration = overrideConfiguration

        expected_ips, motors = destructureMotorCombinationConfig(motorConfiguration)        
        socket.assertConnectedAddresses(expected_ips)

        self.calibrationData = readConfigurationJSON(['config', 'ExoskeletonCalibration.json'])
        calibrationAdjustments = self.calibrationData['adjustments']

        motorConverter = ExoskeletonMotorConverter()

        self.leftAbductor = SafeMotor(motors[0], socket, motorConverter, calibrationAdjustments[0])
        self.rightAbductor = SafeMotor(motors[1], socket, motorConverter, calibrationAdjustments[1])
        self.leftExtensor = SafeMotor(motors[2], socket, motorConverter, calibrationAdjustments[2])
        self.rightExtensor = SafeMotor(motors[3], socket, motorConverter, calibrationAdjustments[3])
        self.leftKnee = SafeMotor(motors[4], socket, motorConverter, calibrationAdjustments[4])
        self.rightKnee = SafeMotor(motors[5], socket, motorConverter, calibrationAdjustments[5])

        self.motorList = [self.leftAbductor, self.rightAbductor, self.leftExtensor, self.rightExtensor, self.leftKnee, self.rightKnee]

        self.dataStream = DataStream(socket, self.motorList, motorConverter)

    def enable(self):
        for motor in self.motorList:
            motor.enable()
        self.dataStream.enable()
    
    def verifyReady(self):
        for motor in self.motorList:
            motor.requestReadyCheck()

        while True:
            readyMotorCount = 0
            for motor in self.motorList:
                if motor.isReady():
                    readyMotorCount += 1

            if readyMotorCount == len(self.motorList):
                break

            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")
    
    def logCalibrationData(self):
        print()
        print(f"Calibration was last done: {self.calibrationData['date']}")
        print(f"Left Abductor Position: {self.leftAbductor.getCVP().position:.4f}")
        print(f"Right Abductor Position: {self.rightAbductor.getCVP().position:.4f}")
        print(f"Left Extensor Position: {self.leftExtensor.getCVP().position:.4f}")
        print(f"Right Extensor Position: {self.rightExtensor.getCVP().position:.4f}")
        print(f"Left Knee Position: {self.leftKnee.getCVP().position:.4f}")
        print(f"Right Knee Position: {self.rightKnee.getCVP().position:.4f}")
        print()        

    def getStreamError(self):
        return self.dataStream.errored()

    def getStreamError(self):
        return super().getStreamError()

    def disable(self):
        for motor in self.motorList:
            motor.setCurrent(0)  
            motor.disable()  
        self.dataStream.disable()

def setup_teardown_exoskeleton(actions: Callable[[Exoskeleton, float], None], totalRunningTime: float):
    setup_teardown_motor_combination(Exoskeleton(), actions, totalRunningTime)

def calibrate_exoskeleton():
    calibrationConfiguration = readConfigurationJSON(["config", "ExoskeletonCalibrationSetup.json"])
    state = CalibrationState(calibrationConfiguration)

    def func(exoskeleton: Exoskeleton, _: float):
        if state.motors is None:
            state.motors = exoskeleton.motorList
        
        state.iterate()

    motorConfiguration = readConfigurationJSON(["config", "Exoskeleton.json"])
    removePositionLimits(motorConfiguration)

    setup_teardown_motor_combination(Exoskeleton(motorConfiguration), func, 120)