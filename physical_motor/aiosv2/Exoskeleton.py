from typing import Callable
from aiosv2.Calibration import CalibrationState
from aiosv2.AiosSocket import AiosSocket
from aiosv2.constants import ExoskeletonMotorConverter, logPosition
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.DataStream import DataStream
from aiosv2.readConfig import readConfigurationJSON, destructureMotorCombinationConfig, removePositionLimits
from aiosv2.ControlLoop import MotorCombination, setup_teardown_motor_combination


class Exoskeleton(MotorCombination):
    def __init__(self, overrideConfiguration: dict | None = None, no_robot=False):
        self.no_robot = no_robot
        socket = AiosSocket(no_robot=self.no_robot)  # Pass the flag to AiosSocket

        motorConfiguration = readConfigurationJSON(["config", "Exoskeleton.json"])
        if overrideConfiguration is not None:
            motorConfiguration = overrideConfiguration

        expected_ips, motors = destructureMotorCombinationConfig(motorConfiguration)
        
        if not self.no_robot:
            socket.assertConnectedAddresses(expected_ips)

        self.calibrationData = readConfigurationJSON(['config', 'ExoskeletonCalibration.json'])
        calibrationAdjustments = self.calibrationData['adjustments']

        motorConverter = ExoskeletonMotorConverter()

        self.leftAbductor = SafeMotor(motors[0], socket, motorConverter, calibrationAdjustments[0], name="Left Abductor")
        self.rightAbductor = SafeMotor(motors[1], socket, motorConverter, calibrationAdjustments[1], name="Right Abductor")
        self.leftExtensor = SafeMotor(motors[2], socket, motorConverter, calibrationAdjustments[2], name="Left Extensor")
        self.rightExtensor = SafeMotor(motors[3], socket, motorConverter, calibrationAdjustments[3], name="Right Extensor")
        self.leftKnee = SafeMotor(motors[4], socket, motorConverter, calibrationAdjustments[4], name="Left Knee")
        self.rightKnee = SafeMotor(motors[5], socket, motorConverter, calibrationAdjustments[5], name="Right Knee")

        self.motorList = [self.leftAbductor, self.rightAbductor, self.leftExtensor, self.rightExtensor, self.leftKnee, self.rightKnee]

        self.dataStream = DataStream(socket, self.motorList, motorConverter)

    def enable(self):
        for motor in self.motorList:
            motor.enable()
        self.dataStream.enable()

    def requestReadyCheck(self):
        for motor in self.motorList:
            motor.requestReadyCheck()
    
    def isReady(self):
        readyMotorCount = 0
        for motor in self.motorList:
            if motor.isReady():
                readyMotorCount += 1

        return readyMotorCount == len(self.motorList)
    
    def logCalibrationData(self):
        print()
        print(f"Calibration was last done: {self.calibrationData['date']}")
        print(f"Left Abductor Position: {logPosition(self.leftAbductor.getCVP().position)}")
        print(f"Right Abductor Position: {logPosition(self.rightAbductor.getCVP().position)}")
        print(f"Left Extensor Position: {logPosition(self.leftExtensor.getCVP().position)}")
        print(f"Right Extensor Position: {logPosition(self.rightExtensor.getCVP().position)}")
        print(f"Left Knee Position: {logPosition(self.leftKnee.getCVP().position)}")
        print(f"Right Knee Position: {logPosition(self.rightKnee.getCVP().position)}")
        print()        

    def getStreamError(self):
        return self.dataStream.errored()

    def disable(self):
        for motor in self.motorList:
            motor.setCurrent(0)  
            motor.disable()  
        self.dataStream.disable()

    def flush(self):
        for motor in self.motorList:
            motor.flush()

def setup_teardown_exoskeleton(actions: Callable[[Exoskeleton, float], None], totalRunningTime: float, no_robot=False):
    setup_teardown_motor_combination(Exoskeleton(), actions, totalRunningTime)

def calibrate_exoskeleton():
    calibrationConfiguration = readConfigurationJSON(["config", "ExoskeletonCalibrationSetup.json"])
    state = CalibrationState(calibrationConfiguration, ["config", "ExoskeletonCalibration.json"], [None, None, None, None, None, None])

    def func(exoskeleton: Exoskeleton, _: float):
        if state.motors is None:
            state.motors = exoskeleton.motorList
        
        state.iterate()

    motorConfiguration = readConfigurationJSON(["config", "Exoskeleton.json"])
    removePositionLimits(motorConfiguration)

    setup_teardown_motor_combination(Exoskeleton(motorConfiguration), func, 120)