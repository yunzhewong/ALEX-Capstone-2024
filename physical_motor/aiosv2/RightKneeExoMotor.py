import time
from typing import Callable, List
from aiosv2.Calibration import CalibrationState
from aiosv2.AiosSocket import AiosSocket
from aiosv2.SafeMotorOperation import (
    SafeMotor
)
from aiosv2.DataStream import DataStream
from aiosv2.constants import ExoskeletonMotorConverter
from aiosv2.readConfig import readConfigurationJSON, destructureMotorCombinationConfig, removePositionLimits

# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY


class RightKneeExoMotor:
    def __init__(self, socket: AiosSocket, configuration: dict, calibrationAdjustments: List[float]):
        self.socket = socket

        expected_ips, motors = destructureMotorCombinationConfig(configuration)

        self.socket.assertConnectedAddresses(expected_ips)
        motorConverter = ExoskeletonMotorConverter()

        self.motor = SafeMotor(motors[0], socket, motorConverter, calibrationAdjustments[0])

        self.dataStream = DataStream(socket, [self.motor], motorConverter)

    def enable(self):
        self.motor.enable()
        self.dataStream.enable()
        
    def verifyReady(self):
        self.motor.requestReadyCheck()

        while not self.motor.isReady():
            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")
    
    def confirmCalibration(self, calibrationInformation: dict):
        print()
        print(f"Calibration was last done: {calibrationInformation['date']}")
        print(f"Motor Position: {self.motor.getCVP().position:.4f}")
        print()        

        result = input("Do you want to continue? (y): ")

        if result != 'y':
            raise Exception("Cancelled by the user")


    def disable(self):
        self.motor.disable()
        self.dataStream.disable()


def setup_teardown_rightknee_exomotor(
    actions: Callable[[RightKneeExoMotor, float], None], totalRunningTime: float, overrideConfiguration: dict | None = None
):
    try:
        socket = AiosSocket()

        motorConfiguration = readConfigurationJSON(["config", "RightKneeExoMotor.json"])
        if overrideConfiguration:
            motorConfiguration = overrideConfiguration
        calibrationInformation = readConfigurationJSON(['config', 'RightKneeCalibration.json'])

        exoMotor = RightKneeExoMotor(socket, motorConfiguration, calibrationInformation['adjustments'])
        exoMotor.enable()

        print("Right Knee Exoskeleton Motor Enabled")

        exoMotor.verifyReady()

        exoMotor.confirmCalibration(calibrationInformation)

        startTime = time.perf_counter()
        currentTime = startTime
        endTime = currentTime + totalRunningTime

        try:
            while currentTime < endTime:
                currentTime = time.perf_counter()
                error = exoMotor.dataStream.errored()
                if error:
                    raise Exception(error)

                runningTime = currentTime - startTime
                actions(exoMotor, runningTime)

                time.sleep(SAMPLING_PERIOD)
        except Exception as e:
            print(e)

        exoMotor.disable()
    except KeyboardInterrupt:
        print("Keyboard Interrupted: Motors turned off")
        print("Keyboard Interrupt again to release locks")
        exoMotor.disable()

def calibrate_right_knee_exomotor():
    calibrationConfiguration = readConfigurationJSON(["config", "RightKneeCalibrationSetup.json"])

    motorConfiguration = readConfigurationJSON(["config", "RightKneeExoMotor.json"])
    _, motors = destructureMotorCombinationConfig(motorConfiguration)
    removePositionLimits(motors)


    state = CalibrationState(calibrationConfiguration)

    def func(exoMotor: RightKneeExoMotor, _: float):
        if state.motors is None:
            state.motors = [exoMotor.motor]
        
        state.iterate()

    setup_teardown_rightknee_exomotor(func, 120, motorConfiguration)