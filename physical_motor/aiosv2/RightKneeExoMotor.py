import math
import time
from typing import Callable
from aiosv2 import AiosSocket
from aiosv2.SafeMotorOperation import (
    SafeMotor,
    SafetyConfiguration,
    SafetyLimit,
    SafetyValueRange,
)
from aiosv2.DataStream import DataStream
from aiosv2.constants import ExoskeletonMotorConverter

# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY


class RightKneeExoMotor:
    CONTROL_BOX = "10.10.10.12"
    MOTOR_IP = "10.10.10.30"
    EXPECTED_IPS = [CONTROL_BOX, MOTOR_IP]

    def __init__(self, socket: AiosSocket):
        self.socket = socket

        self.socket.assertConnectedAddresses(self.EXPECTED_IPS)
        motorConverter = ExoskeletonMotorConverter()

        config = SafetyConfiguration(
            current_limit=SafetyLimit(
                "Current", SafetyValueRange(-100, 100), SafetyValueRange(-15, 15)
            ),
            # velocity_limit=SafetyLimit(
            #     "Velocity",
            #     SafetyValueRange(-1, 1),
            #     SafetyValueRange(-4, 4),
            # ),
            velocity_limit=SafetyLimit(
                "Velocity",
                SafetyValueRange(-3.5 * math.pi, 3.5 * math.pi),
                SafetyValueRange(-4 * math.pi, 4 * math.pi),
            ),
            position_limit=SafetyLimit(
                "Position",
                SafetyValueRange(-math.pi, math.pi),
                SafetyValueRange(-2 * math.pi, 2 * math.pi),
            ),
            # position_limit=SafetyLimit(
            #     "Position",
            #     SafetyValueRange(-10 * math.pi, 10 * math.pi),
            #     SafetyValueRange(-15 * math.pi, 15 * math.pi),
            # ),
        )
        self.motor = SafeMotor(self.MOTOR_IP, socket, config, motorConverter)

        self.dataStream = DataStream(socket, [self.motor], motorConverter)

    def enable(self):
        self.motor.enable()
        self.dataStream.enable()

    def disable(self):
        self.motor.disable()
        self.dataStream.disable()


def setup_teardown_rightknee_exomotor(
    actions: Callable[[RightKneeExoMotor, float], None], totalRunningTime: float
):
    try:
        socket = AiosSocket()
        exoMotor = RightKneeExoMotor(socket)
        exoMotor.enable()

        exoMotor.motor.requestEncoderReady()

        while not exoMotor.motor.encoderIsReady():
            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")

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
