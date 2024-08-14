import datetime
import math
import time
from typing import Callable, List
from aiosv2.Calibration import CalibrationState
from aiosv2 import AiosSocket
from aiosv2.constants import TwinMotorConverter
from aiosv2.SafeMotorOperation import (
    SafeMotor,
)
from aiosv2.DataStream import DataStream
from aiosv2.readConfig import readConfigurationJSON, removePositionLimits, destructureMotorCombinationConfig, writeConfigurationJSON

# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY


class TwinMotor:
    def __init__(self, socket: AiosSocket, configuration: dict, calibrationAdjustments: List[float]):
        self.socket = socket

        expected_ips, motors = destructureMotorCombinationConfig(configuration)
        
        self.socket.assertConnectedAddresses(expected_ips)
        motorConverter = TwinMotorConverter()

        self.topMotor = SafeMotor(motors[0], socket, motorConverter, 0)
        self.bottomMotor = SafeMotor(motors[1], socket, motorConverter, calibrationAdjustments[0])
        self.dataStream = DataStream(
            socket, [self.topMotor, self.bottomMotor], motorConverter
        )

    def enable(self):
        self.topMotor.enable()  # Enable the top motor
        self.bottomMotor.enable()  # Enable the bottom motor
        self.dataStream.enable()

    def verifyReady(self):
        self.bottomMotor.requestReadyCheck()
        self.topMotor.requestReadyCheck()

        while not (self.topMotor.isReady() and self.bottomMotor.isReady()):
            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")
    
    def confirmCalibration(self, calibrationInformation: dict):
        print()
        print(f"Calibration was last done: {calibrationInformation['date']}")
        print(f"Top Motor Position: {self.topMotor.getCVP().position:.4f}")
        print(f"Bottom Motor Position: {self.bottomMotor.getCVP().position:.4f}")
        print()        

        result = input("Do you want to continue? (y): ")

        if result != 'y':
            raise Exception("Cancelled by the user")

    def disable(self):
        self.topMotor.disable()  # Disable the top motor
        self.bottomMotor.disable()  # Disable the bottom motor
        self.dataStream.disable()


def setup_teardown_twin_motor(
    actions: Callable[[TwinMotor, float], None], totalRunningTime: float, overrideConfiguration: None | dict=None
):
    try:
        socket = AiosSocket()
        configuration = readConfigurationJSON(["config", "TwinMotor.json"])
        if overrideConfiguration is not None:
            configuration = overrideConfiguration

        calibrationInformation = readConfigurationJSON(['config', 'TwinMotorCalibration.json'])

        twinMotor = TwinMotor(socket, configuration, calibrationInformation['adjustments'])
        twinMotor.enable()

        print("Twin Motor Enabled")

        twinMotor.verifyReady()

        twinMotor.confirmCalibration(calibrationInformation)

        startTime = time.perf_counter()
        currentTime = startTime
        endTime = currentTime + totalRunningTime

        try:
            while currentTime < endTime:
                currentTime = time.perf_counter()
                error = twinMotor.dataStream.errored()
                if error:
                    raise Exception(error)

                runningTime = currentTime - startTime
                actions(twinMotor, runningTime)

                time.sleep(SAMPLING_PERIOD)
        except Exception as e:
            print(e)

        twinMotor.disable()
    except KeyboardInterrupt:
        print("Keyboard Interrupted: Motors turned off")
        print("Keyboard Interrupt again to release locks")
        twinMotor.disable()


def calibrate_twin_motor():

    calibrationConfiguration = readConfigurationJSON(["config", "TwinMotorCalibrationSetup.json"])

    motorConfiguration = readConfigurationJSON(["config", "TwinMotor.json"])
    _, motors = destructureMotorCombinationConfig(motorConfiguration)
    removePositionLimits(motors)


    state = CalibrationState(calibrationConfiguration)

    def func(twinMotor: TwinMotor, _: float):
        if state.motors is None:
            state.motors = [twinMotor.bottomMotor]
        
        state.iterate()

    setup_teardown_twin_motor(func, 120, motorConfiguration)
    