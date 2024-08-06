import time
from typing import Callable
from aiosv2.AiosSocket import AiosSocket
from aiosv2.SafeMotorOperation import (
    SafeMotor
)
from aiosv2.DataStream import DataStream
from aiosv2.constants import ExoskeletonMotorConverter
from aiosv2.readConfig import readConfig

# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY


class RightKneeExoMotor:
    def __init__(self, socket: AiosSocket):
        self.socket = socket

        expected_ips, motors = readConfig("RightKneeExoMotor.json")
        self.socket.assertConnectedAddresses(expected_ips)
        motorConverter = ExoskeletonMotorConverter()

        self.motor = SafeMotor(motors[0], socket, motorConverter)

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

        exoMotor.motor.requestReadyCheck()

        while not exoMotor.motor.isReady():
            print("Checking Encoder Status and Reading CVP...")
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
