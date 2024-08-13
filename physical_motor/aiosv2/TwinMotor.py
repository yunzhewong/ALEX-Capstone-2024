import time
from typing import Callable
from aiosv2 import AiosSocket
from aiosv2.constants import TwinMotorConverter
from aiosv2.SafeMotorOperation import (
    SafeMotor,
)
from aiosv2.DataStream import DataStream
from aiosv2.readConfig import readConfig, prepareConfigurationForCalibration

# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY


class TwinMotor:
    def __init__(self, socket: AiosSocket):
        self.socket = socket

        expected_ips, motors = readConfig(["config", "TwinMotor.json"])
        
        self.socket.assertConnectedAddresses(expected_ips)
        motorConverter = TwinMotorConverter()

        self.topMotor = SafeMotor(motors[0], socket, motorConverter)
        self.bottomMotor = SafeMotor(motors[1], socket, motorConverter)
        self.dataStream = DataStream(
            socket, [self.topMotor, self.bottomMotor], motorConverter
        )

    def enable(self):
        self.topMotor.enable()  # Enable the top motor
        self.bottomMotor.enable()  # Enable the bottom motor
        self.dataStream.enable()

    def disable(self):
        self.topMotor.disable()  # Disable the top motor
        self.bottomMotor.disable()  # Disable the bottom motor
        self.dataStream.disable()


def setup_teardown_twin_motor(
    actions: Callable[[TwinMotor, float], None], totalRunningTime: float
):
    try:
        socket = AiosSocket()
        twinMotor = TwinMotor(socket)
        twinMotor.enable()

        twinMotor.bottomMotor.requestReadyCheck()
        twinMotor.topMotor.requestReadyCheck()

        while not twinMotor.topMotor.isReady() or not twinMotor.bottomMotor.isReady():
            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")

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

class CalibrationTwinMotor():
    def __init__(self, socket: AiosSocket):
        self.socket = socket

        expected_ips, motors = readConfig(["config", "TwinMotor.json"])
        prepareConfigurationForCalibration(motors)
        
        self.socket.assertConnectedAddresses(expected_ips)
        motorConverter = TwinMotorConverter()

        self.topMotor = SafeMotor(motors[0], socket, motorConverter)
        self.bottomMotor = SafeMotor(motors[1], socket, motorConverter)
        self.dataStream = DataStream(
            socket, [self.topMotor, self.bottomMotor], motorConverter
        )

    def enable(self):
        self.topMotor.enable()  # Enable the top motor
        self.bottomMotor.enable()  # Enable the bottom motor
        self.dataStream.enable()

    def disable(self):
        self.topMotor.disable()  # Disable the top motor
        self.bottomMotor.disable()  # Disable the bottom motor
        self.dataStream.disable()


def calibrate_twin_motor():
    try:
        socket = AiosSocket()
        twinMotor = CalibrationTwinMotor(socket)
        twinMotor.enable()

        twinMotor.bottomMotor.requestReadyCheck()
        twinMotor.topMotor.requestReadyCheck()

        while not twinMotor.topMotor.isReady() or not twinMotor.bottomMotor.isReady():
            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")

        startTime = time.perf_counter()
        currentTime = startTime
        endTime = currentTime + 120

        motors = [twinMotor.bottomMotor, twinMotor.topMotor]
        zeroVelocitiesCounts = [0, 0]

        try:
            while currentTime < endTime:
                currentTime = time.perf_counter()
                error = twinMotor.dataStream.errored()
                if error:
                    raise Exception(error)

                for motor in motors:
                    cvp = motor.getCVP()
                    
                    motor.setVelocity(0.3)

                time.sleep(SAMPLING_PERIOD)
        except Exception as e:
            print(e)

        twinMotor.disable()
    except KeyboardInterrupt:
        print("Keyboard Interrupted: Motors turned off")
        print("Keyboard Interrupt again to release locks")
        twinMotor.disable()