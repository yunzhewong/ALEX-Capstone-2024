import math
import time
from typing import Callable, List
from aiosv2 import AiosSocket
from aiosv2.constants import TwinMotorConverter
from aiosv2.SafeMotorOperation import (
    SafeMotor,
)
from aiosv2.DataStream import DataStream
from aiosv2.readConfig import readConfigurationJSON, prepareConfigurationForCalibration, destructureMotorCombinationConfig

# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY


class TwinMotor:
    def __init__(self, socket: AiosSocket, configuration: dict):
        self.socket = socket

        expected_ips, motors = destructureMotorCombinationConfig(configuration)
        
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
    actions: Callable[[TwinMotor, float], None], totalRunningTime: float, overrideConfiguration: None | dict=None
):
    try:
        socket = AiosSocket()
        configuration = readConfigurationJSON(["config", "TwinMotor.json"])
        if overrideConfiguration is not None:
            configuration = overrideConfiguration

        twinMotor = TwinMotor(socket, configuration)
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

EPSILON = 0.01



class CalibrationState():
    def __init__(self):
        self.motors: List[SafeMotor] | None = None
        self.desiredPosition = [math.pi]
        self.calibrationPosition = [None]
        self.zeroVelocitiesCounts = [0]

def calibrate_twin_motor():

    state = CalibrationState()

    def func(twinMotor: TwinMotor, runningTime: float):
        if state.motors is None:
            state.motors = [twinMotor.bottomMotor]

        for i, motor in enumerate(state.motors):
            cvp = motor.getCVP()

            if cvp.velocity < EPSILON:
                state.zeroVelocitiesCounts[i] += 1

            if state.zeroVelocitiesCounts[i] > 10 and state.calibrationPosition[i] is None:
                print(f"{motor.getIP()} Found Limit")
                state.calibrationPosition[i] = cvp.position

            if state.calibrationPosition[i] is None:
                motor.setVelocity(0.3)
            else:
                position_adjustment = state.desiredPosition[i] - state.calibrationPosition[i]
                truePosition = cvp.position + position_adjustment
                print(truePosition)
                if abs(truePosition) > EPSILON:
                    motor.setVelocity(-0.3)
                else:
                    motor.setVelocity(0)


                    calibrated = 0
                    for pos in state.calibrationPosition:
                        if pos is not None:
                            calibrated += 1

                    if calibrated == len(state.calibrationPosition):
                        raise Exception("All Motors Calibrated")

        
    configuration = readConfigurationJSON(["config", "TwinMotor.json"])
    _, motors = destructureMotorCombinationConfig(configuration)
    prepareConfigurationForCalibration(motors)

    setup_teardown_twin_motor(func, 120, configuration)
    