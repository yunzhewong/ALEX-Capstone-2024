import datetime
import math
import time
from typing import Callable, List
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

    def requestReadyCheck(self):
        self.bottomMotor.requestReadyCheck()
        self.topMotor.requestReadyCheck()

    def isReady(self):
        return self.topMotor.isReady() and self.bottomMotor.isReady()
    
    def logCalibrationData(self, calibrationInformation: dict):
        print()
        print(f"Calibration was last done: {calibrationInformation['date']}")
        print(f"Top Motor Position: {self.topMotor.getCVP().position:.4f}")
        print(f"Bottom Motor Position: {self.bottomMotor.getCVP().position:.4f}")
        print()        

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

        twinMotor.requestReadyCheck()

        while not twinMotor.isReady():
            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")

        twinMotor.logCalibrationData(calibrationInformation)
        
        result = input("Do you want to continue? (y): ")

        if result != 'y':
            raise Exception("Cancelled by the user")
         
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

EPSILON = 0.01
CALIBRATION_SPEED = 0.3
ZERO_VELOCITY_COUNT = 10

class MotorCalibration():
    def __init__(self, config: dict):
        print(config)
        self.direction = config['direction']
        self.limit = config['limit']
        self.readLimitPosition: float | None = None
        self.zeroVelocityCount = 0


class CalibrationState():
    def __init__(self, setupConfig: dict):
        self.motors: List[SafeMotor] | None = None
        self.calibrations = [MotorCalibration(config) for config in setupConfig['motors']]

    def allMotorsCalibrated(self):
        count = 0 
        for calibration in self.calibrations:
            if calibration.readLimitPosition is not None:
                count += 1
        
        return count == len(self.calibrations)
    
    def iterate(self):
        for i, motor in enumerate(self.motors):
            cvp = motor.getNonCalibratedCVP()

            if abs(cvp.velocity) < EPSILON:
                self.calibrations[i].zeroVelocityCount += 1

            atBoundary = self.calibrations[i].zeroVelocityCount > ZERO_VELOCITY_COUNT
            positionNotFoundYet = self.calibrations[i].readLimitPosition is None  

            if atBoundary and positionNotFoundYet:
                print(f"{motor.getIP()} Found Limit")
                self.calibrations[i].readLimitPosition = cvp.position

            calibrationVelocity = self.calibrations[i].direction * CALIBRATION_SPEED

            if self.calibrations[i].readLimitPosition is None:
                motor.setVelocity(calibrationVelocity)
            else:
                position_adjustment = self.calibrations[i].limit - self.calibrations[i].readLimitPosition
                truePosition = cvp.position + position_adjustment
                if abs(truePosition) > EPSILON:
                    motor.setVelocity(-1 * calibrationVelocity)
                else:
                    motor.setVelocity(0)

                    if self.allMotorsCalibrated():
                        writeConfigurationJSON({
                            "date": datetime.datetime.now().isoformat(),
                            "adjustments": [self.calibrations[i].limit - self.calibrations[i].readLimitPosition for i in range(len(self.calibrations))]
                        }, ["config", "TwinMotorCalibration.json"])
                        raise Exception("All Motors Calibrated")


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
    