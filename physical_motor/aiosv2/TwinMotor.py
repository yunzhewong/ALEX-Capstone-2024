import math
import time
from typing import Callable
from aiosv2 import AiosSocket
from aiosv2.constants import TwinMotorConverter
from aiosv2.SafeMotorOperation import (
    SafeMotor,
    SafetyConfiguration,
    SafetyLimit,
    SafetyValueRange,
)
from aiosv2.DataStream import DataStream

# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY


class TwinMotor:
    CONTROL_BOX = "10.10.10.12"
    MOTORS = {"top": "10.10.10.16", "bottom": "10.10.10.17"}
    EXPECTED_IPS = [CONTROL_BOX] + list(MOTORS.values())

    def __init__(self, socket: AiosSocket):
        self.socket = socket

        self.socket.assertConnectedAddresses(self.EXPECTED_IPS)
        motorConverter = TwinMotorConverter()

        topConfig = SafetyConfiguration(
            current_limit=SafetyLimit(
                "Current", SafetyValueRange(-100, 100), SafetyValueRange(-15, 15)
            ),
            velocity_limit=SafetyLimit(
                "Velocity",
                SafetyValueRange(-3.5 * math.pi, 3.5 * math.pi),
                SafetyValueRange(-4 * math.pi, 4 * math.pi),
            ),
            position_limit=SafetyLimit(
                "Position",
                SafetyValueRange(-15 * math.pi, 15 * math.pi),
                SafetyValueRange(-20 * math.pi, 20 * math.pi),
            ),
        )
        self.topMotor = SafeMotor(self.MOTORS["top"], socket, topConfig, motorConverter)

        bottomConfig = SafetyConfiguration(
            current_limit=SafetyLimit(
                "Current", SafetyValueRange(-100, 100), SafetyValueRange(-15, 15)
            ),
            velocity_limit=SafetyLimit(
                "Velocity",
                SafetyValueRange(-3.5 * math.pi, 3.5 * math.pi),
                SafetyValueRange(-4 * math.pi, 4 * math.pi),
            ),
            position_limit=SafetyLimit(
                "Position",
                SafetyValueRange(-math.pi / 2, math.pi / 2),
                SafetyValueRange(-2 * math.pi / 3, 2 * math.pi / 3),
            ),
        )
        self.bottomMotor = SafeMotor(
            self.MOTORS["bottom"], socket, bottomConfig, motorConverter
        )
        self.dataStream = DataStream(
            socket, [self.topMotor, self.bottomMotor], motorConverter
        )

    def enable(self):
        self.topMotor.enable()  # Enable the top motor
        self.bottomMotor.enable()  # Enable the bottom motor
        self.dataStream.enable()

    def disable(self):
        self.topMotor.setCurrent(0)  # Stop the top motor
        self.bottomMotor.setCurrent(0)  # Stop the bottom motor
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

        twinMotor.bottomMotor.requestEncoderReady()
        twinMotor.topMotor.requestEncoderReady()

        while (
            not twinMotor.topMotor.encoderIsReady()
            or not twinMotor.bottomMotor.encoderIsReady()
        ):
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
