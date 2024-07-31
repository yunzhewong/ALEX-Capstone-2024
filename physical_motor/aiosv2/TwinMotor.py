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
import json
from aiosv2.DataStream import DataStream

# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY


class TwinMotor:
    def __init__(self, socket: AiosSocket):
        self.socket = socket

        config_file = open("./config/TwinMotor.json")
        configuration = json.loads(config_file.read())
        print(configuration)

        control_box_ip = configuration["control_box_ip"]
        motors = configuration["motors"]

        expected_ips = [control_box_ip] + [motor["ip"] for motor in motors]

        self.socket.assertConnectedAddresses(expected_ips)
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
        self.topMotor = SafeMotor(motors[0]["ip"], socket, topConfig, motorConverter)

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
            motors[1]["ip"], socket, bottomConfig, motorConverter
        )
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
