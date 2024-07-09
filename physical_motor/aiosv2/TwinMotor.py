import math
import time
from typing import Callable
from aiosv2 import AiosSocket, ConnectedMotor
from aiosv2.SafeMotorOperation import SafeMotor, SafetyConfiguration
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

        topConfig = SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=4*math.pi, minimum_position=-15 * math.pi, maximum_position=15 * math.pi)
        self.topMotor = SafeMotor(self.MOTORS["top"], socket, topConfig)

        bottomConfig = SafetyConfiguration(margin=0.05, maximum_current=15, maximum_velocity=4*math.pi, minimum_position=-2 * math.pi / 3, maximum_position=2 * math.pi / 3) 
        self.bottomMotor = SafeMotor(self.MOTORS["bottom"], socket, bottomConfig)
        self.dataStream = DataStream(socket, [self.topMotor, self.bottomMotor])

    def enable(self):
        self.topMotor.enable()  # Enable the top motor
        self.bottomMotor.enable()  # Enable the bottom motor
        self.dataStream.enable()

    def disable(self):
        self.topMotor.setCurrent(0) # Stop the top motor
        self.bottomMotor.setCurrent(0) # Stop the bottom motor
        self.topMotor.disable()  # Disable the top motor
        self.bottomMotor.disable()  # Disable the bottom motor
        self.dataStream.disable()

def setup_teardown_twin_motor(actions: Callable[[TwinMotor, float], None], totalRunningTime: float):
    try:
        socket = AiosSocket()
        twinMotor = TwinMotor(socket)
        twinMotor.enable()

        twinMotor.bottomMotor.requestEncoderReady()
        twinMotor.topMotor.requestEncoderReady()

        while not twinMotor.topMotor.encoderIsReady() or not twinMotor.bottomMotor.encoderIsReady():
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
