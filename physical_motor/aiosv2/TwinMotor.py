import math
import time
from typing import Callable
from aiosv2 import AiosSocket, ConnectedMotor
from aiosv2.SafeMotorOperation import SafeMotor, SafetyConfiguration
from aiosv2.CVPStream import CVPStream

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
        config = SafetyConfiguration(maximum_current=20, maximum_velocity=4 * math.pi, minimum_position=-math.pi /2, maximum_position=math.pi / 2)
        self.topMotor = SafeMotor(self.MOTORS["top"], socket, config)
        self.bottomMotor = SafeMotor(self.MOTORS["bottom"], socket, config)
        self.cvpStream = CVPStream(socket, [self.topMotor, self.bottomMotor])

    def enable(self):
        self.topMotor.enable()  # Enable the top motor
        self.bottomMotor.enable()  # Enable the bottom motor
        self.cvpStream.enable()

    def disable(self):
        self.topMotor.disable()  # Disable the top motor
        self.bottomMotor.disable()  # Disable the bottom motor
        self.cvpStream.disable()

    def on_error(self):
        self.topMotor.setCurrent(0)  # Stop the top motor
        self.bottomMotor.setCurrent(0)  # Stop the bottom motor
        self.cvpStream.disable()

def setup_teardown_twin_motor(actions: Callable[[TwinMotor, float], None], totalRunningTime: float):
    socket = AiosSocket()
    twinMotor = TwinMotor(socket)
    twinMotor.enable()

    startTime = time.perf_counter()
    currentTime = startTime
    endTime = currentTime + totalRunningTime

    try:
        while currentTime < endTime:
            currentTime = time.perf_counter()
            error = twinMotor.cvpStream.errored() 
            if error:
                raise Exception(error)
            
            runningTime = currentTime - startTime
            actions(twinMotor, runningTime)
            
            time.sleep(SAMPLING_PERIOD)
    except Exception as e:
        print(e)
        twinMotor.on_error()

    twinMotor.disable()
