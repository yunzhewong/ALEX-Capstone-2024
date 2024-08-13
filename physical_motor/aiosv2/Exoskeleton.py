import math
import time
from typing import Callable
from aiosv2.AiosSocket import AiosSocket
from aiosv2.constants import ExoskeletonMotorConverter, TwinMotorConverter
from aiosv2.SafeMotorOperation import SafeMotor, SafetyConfiguration
from aiosv2.DataStream import DataStream
from aiosv2.readConfig import readConfigurationJSON, destructureMotorCombinationConfig

SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY

class Exoskeleton:
    def __init__(self, socket: AiosSocket):
        self.socket = socket

        configurationJSON = readConfigurationJSON(["config", "Exoskeleton.json"])
        expected_ips, motors = destructureMotorCombinationConfig(configurationJSON)
        
        self.socket.assertConnectedAddresses(expected_ips)
        motorConverter = ExoskeletonMotorConverter()

        self.leftAbductor = SafeMotor(motors[0], socket, motorConverter)
        self.rightAbductor = SafeMotor(motors[1], socket, motorConverter)
        self.leftExtensor = SafeMotor(motors[2], socket, motorConverter)
        self.rightExtensor = SafeMotor(motors[3], socket, motorConverter)
        self.leftKnee = SafeMotor(motors[4], socket, motorConverter)
        self.rightKnee = SafeMotor(motors[5], socket, motorConverter)

        self.motorList = [self.leftAbductor, self.rightAbductor, self.leftExtensor, self.rightExtensor, self.leftKnee, self.rightKnee]

        self.dataStream = DataStream(socket, self.motorList, motorConverter)

    def enable(self):
        for motor in self.motorList:
            motor.enable()
        self.dataStream.enable()

    def disable(self):
        for motor in self.motorList:
            motor.setCurrent(0)  
            motor.disable()  
        self.dataStream.disable()

def setup_teardown_exoskeleton(actions: Callable[[Exoskeleton, float], None], totalRunningTime: float):
    try:
        socket = AiosSocket()
        exoskeleton = Exoskeleton(socket)
        exoskeleton.enable()

        for motor in exoskeleton.motorList:
            motor.requestReadyCheck()


        # needs to be modified when test with only one motor. 
        while not all(motor.isReady() for motor in exoskeleton.motorList):
            print("Checking Encoder Status...")
            time.sleep(0.1)
        print("Encoder Ready")

        startTime = time.perf_counter()
        currentTime = startTime
        endTime = currentTime + totalRunningTime

        try:
            while currentTime < endTime:
                currentTime = time.perf_counter()
                error = exoskeleton.dataStream.errored()
                if error:
                    raise Exception(error)

                runningTime = currentTime - startTime
                actions(exoskeleton, runningTime)

                time.sleep(SAMPLING_PERIOD)
        except Exception as e:
            print(e)

        exoskeleton.disable()
    except KeyboardInterrupt:
        print("Keyboard Interrupted: Motors turned off")
        print("Keyboard Interrupt again to release locks")
        exoskeleton.disable()
