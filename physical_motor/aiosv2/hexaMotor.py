import math
import time
from typing import Callable
from aiosv2 import AiosSocket
from aiosv2.constants import TwinMotorConverter
from aiosv2.SafeMotorOperation import SafeMotor, SafetyConfiguration
from aiosv2.DataStream import DataStream

SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY

class HexaMotor:
    # motor IP needs to be confirmed
    CONTROL_BOX = "10.10.10.12"
    MOTORS = {
        "hipPitchLeft": "10.10.10.16",
        "hipPitchRight": "10.10.10.17",
        "hipRollLeft": "10.10.10.18",
        "hipRollRight": "10.10.10.19",
        "kneeLeft": "10.10.10.20",
        "kneeRight": "10.10.10.21"
    }
    EXPECTED_IPS = [CONTROL_BOX] + list(MOTORS.values())

    def __init__(self, socket: AiosSocket):
        self.socket = socket
        self.socket.assertConnectedAddresses(self.EXPECTED_IPS)
        motorConverter = TwinMotorConverter()  # convert rate needs to be confirmed.

        config = {
            "hipPitchLeft": SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=2 * math.pi, minimum_position=-math.pi / 2, maximum_position=math.pi / 2),
            "hipPitchRight": SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=2 * math.pi, minimum_position=-math.pi / 2, maximum_position=math.pi / 2),
            "hipRollLeft": SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=2 * math.pi, minimum_position=-math.pi / 2, maximum_position=math.pi / 2),
            "hipRollRight": SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=2 * math.pi, minimum_position=-math.pi / 2, maximum_position=math.pi / 2),
            "kneeLeft": SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=2 * math.pi, minimum_position=-math.pi / 2, maximum_position=math.pi / 2),
            "kneeRight": SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=2 * math.pi, minimum_position=-math.pi / 2, maximum_position=math.pi / 2),
        }

        # Initial setting of all motors.
        self.motors = {name: SafeMotor(ip, socket, config[name], motorConverter) for name, ip in self.MOTORS.items()}
        self.dataStream = DataStream(socket, list(self.motors.values()), motorConverter)

    def enable(self):
        for motor in self.motors.values():
            motor.enable()
        self.dataStream.enable()

    def disable(self):
        for motor in self.motors.values():
            motor.setCurrent(0)  # Stop the motor
            motor.disable()  # Disable the bottom motor
        self.dataStream.disable()

def setup_teardown_hexa_motor(actions: Callable[[HexaMotor, float], None], totalRunningTime: float):
    try:
        socket = AiosSocket()
        hexaMotor = HexaMotor(socket)
        hexaMotor.enable()

        for motor in hexaMotor.motors.values():
            motor.requestEncoderReady()


        # needs to be modified when test with only one motor. 
        while not all(motor.encoderIsReady() for motor in hexaMotor.motors.values()):
            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")

        startTime = time.perf_counter()
        currentTime = startTime
        endTime = currentTime + totalRunningTime

        try:
            while currentTime < endTime:
                currentTime = time.perf_counter()
                error = hexaMotor.dataStream.errored()
                if error:
                    raise Exception(error)

                runningTime = currentTime - startTime
                actions(hexaMotor, runningTime)

                time.sleep(SAMPLING_PERIOD)
        except Exception as e:
            print(e)

        hexaMotor.disable()
    except KeyboardInterrupt:
        print("Keyboard Interrupted: Motors turned off")
        print("Keyboard Interrupt again to release locks")
        hexaMotor.disable()
