from enum import Enum
import threading
from typing import List
from aiosv2.AiosSocket import AiosSocket
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.ConnectedMotor import CVPConverter
from aiosv2.CVP import CVP
from aiosv2.constants import Converter


class DataType(Enum):
    CVP = ("CVP",)
    ERROR = "ERROR"
    ENCODER = "ENCODER"
    MOTION_CONFIG = "MOTION_CONFIG"


class PIDConfig:
    def __init__(self, response, converter: Converter):
        self.positionP = response.get("pos_gain")
        self.velocityP = response.get("vel_gain")
        self.velocityI = response.get("vel_integrator_gain")
        self.velocityLimit = response.get("vel_limit")
        self.limitTolerance = response.get("vel_limit_tolerance")
        self.motorConverter = converter

    def __str__(self):
        if self.velocityLimit is None:
            return "Motion Control Config Set"
        return f"Position Gain: {self.positionP}, Velocity Gain: {self.velocityP}, Velocity Int: {self.velocityI}, Velocity Limit: {self.velocityLimit} ({self.motorConverter.convertFromMotorCommand(self.velocityLimit)}), Limit Tolerance: {self.limitTolerance}"


class ErrorParser:
    NO_ERROR = "ERROR_NONE"

    def check_for_error(self, json_obj):
        axisError = json_obj.get("axis")
        motorError = json_obj.get("motor")
        encoderError = json_obj.get("encoder")

        if axisError != self.NO_ERROR:
            raise Exception(f"Error with the motor axis: {axisError}")

        if motorError != self.NO_ERROR:
            raise Exception(f"Error with the motor: {motorError}")

        if encoderError != self.NO_ERROR:
            raise Exception(f"Error with the encoder: {encoderError}")


class DataStream:
    def __init__(
        self, socket: AiosSocket, motors: List[SafeMotor], motorConverter: Converter
    ):
        self.socket = socket
        self.motors = motors
        self.stop = False
        self.error = None
        self.readThread = None
        self.requestThread = None
        self.cvpConverter = CVPConverter(motorConverter)
        self.motorConverter = motorConverter
        self.errorParser = ErrorParser()

    def enable(self):
        self.readThread = threading.Thread(target=self.read)
        self.readThread.start()

    def disable(self):
        self.stop = True
        self.readThread.join()

    def errored(self):
        return self.error

    def read(self):
        while True:
            if self.stop:
                break

            result = self.check_for_data()
            if result is None:
                continue

            try:
                json_obj, ip, datatype = result

                if datatype == DataType.CVP:
                    for motor in self.motors:
                        if motor.getIP() == ip:
                            cvp = self.cvpConverter.parseCVP(json_obj)
                            motor.setCVP(cvp)
                elif datatype == DataType.ERROR:
                    self.errorParser.check_for_error(json_obj)
                elif datatype == DataType.ENCODER:
                    ready = json_obj.get("property", None)
                    if ready is None or not ready:
                        return

                    for motor in self.motors:
                        if motor.getIP() == ip:
                            motor.confirmEncoderReady()
                elif datatype == DataType.MOTION_CONFIG:
                    data = PIDConfig(json_obj, self.motorConverter)
                    print(json_obj)
                else:
                    raise Exception("This should not happen")
            except Exception as err:
                self.stop = True
                self.error = err

    def check_for_data(self):
        try:
            json_obj, ip = self.socket.readJSON()
            print(json_obj)
            target = json_obj.get("reqTarget")
            if target in ["/m1/setPosition", "/m1/setVelocity", "/m1/setCurrent"]:
                return json_obj, ip, DataType.CVP
            if target in ["/m1/error"]:
                return json_obj, ip, DataType.ERROR
            if target == "/m1/encoder/is_ready":
                return json_obj, ip, DataType.ENCODER
            if target == "/m1/controller/config":
                return json_obj, ip, DataType.MOTION_CONFIG
            return None
        except:
            return None
