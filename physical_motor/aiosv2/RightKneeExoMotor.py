import math
import time
from typing import Callable
from aiosv2 import AiosSocket
from aiosv2.SafeMotorOperation import SafeMotor, SafetyConfiguration, SafetyLimit
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
            current_limit=SafetyLimit("Current", -15, 15),
            velocity_limit=SafetyLimit("Velocity", -4 * math.pi, 4 * math.pi),
            position_limit=SafetyLimit("Position", -15 * math.pi, 15 * math.pi),
        )
        self.motor = SafeMotor(self.MOTOR_IP, socket, config, motorConverter)

        self.dataStream = DataStream(socket, [self.motor], motorConverter)

    def enable(self):
        self.motor.enable()
        self.dataStream.enable()

    def disable(self):
        self.motor.setCurrent(0)
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
