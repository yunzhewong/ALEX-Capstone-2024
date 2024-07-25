import socket
import struct
from aiosv2.constants import (
    ControlMode,
    AxisState,
    PORT_rt,
    PORT_srv,
)
from aiosv2.CVP import CVP
from aiosv2.AiosSocket import AiosSocket
from aiosv2.constants import Converter


class CVPConverter:
    def __init__(self, motorConverter: Converter):
        self.motorConverter = motorConverter

    def parseCVP(self, json_obj):
        readStatus = json_obj.get("status")
        readPosition = json_obj.get("position")
        readVelocity = json_obj.get("velocity")
        readCurrent = json_obj.get("current")

        validStatus = readStatus == "OK"
        validPosition = readPosition is not None
        validVelocity = readVelocity is not None
        validCurrent = readCurrent is not None

        if (
            not validStatus
            or not validPosition
            or not validVelocity
            or not validCurrent
        ):
            raise Exception("Invalid CVP")

        position = self.motorConverter.convertFromMotorCommand(readPosition)
        velocity = self.motorConverter.convertFromMotorCommand(readVelocity)
        current = readCurrent
        return CVP(current, velocity, position)


# Represents a motor connected to the system
# Has methods to request and get the Current, Velocity and Position of the motor
class ConnectedMotor:
    def __init__(self, ip: str, socket: AiosSocket, motorConverter: Converter):
        self.ip = ip
        self.socket = socket
        self.controlMode = ControlMode.Current
        self.cvpConverter = CVPConverter(motorConverter)
        self.motorConverter = motorConverter

    def enable(self):
        self.socket.changeState(self.ip, AxisState.AXIS_STATE_ENABLE.value)

    def disable(self):
        self.socket.changeState(self.ip, AxisState.AXIS_STATE_IDLE.value)

    def requestCVP(self):
        self.socket.sendJSON(
            self.ip,
            PORT_rt,
            {
                "method": "GET",
                "reqTarget": "/m1/CVP",
            },
        )

    def requestEncoderCheck(self):
        self.socket.sendJSON(
            self.ip,
            PORT_srv,
            {
                "method": "GET",
                "reqTarget": "/m1/encoder/is_ready",
            },
        )

    def requestErrorCheck(self):
        self.socket.sendJSON(
            self.ip,
            PORT_srv,
            {
                "method": "GET",
                "reqTarget": "/m1/error",
            },
        )

    def setControlMode(self, mode: ControlMode):
        self.socket.sendJSON(
            self.ip,
            PORT_rt,
            {
                "method": "SET",
                "reqTarget": "/m1/controller/config",
                "control_mode": mode.value,
                "reply_enable": False,
            },
        )

    def setPosition(self, position: float, velocity_ff=0, current_ff=0):
        positionCommand = self.motorConverter.convertToMotorCommand(position)
        print(positionCommand)
        self.socket.sendJSON(
            self.ip,
            PORT_rt,
            {
                "method": "SET",
                "reqTarget": "/m1/setPosition",
                "position": positionCommand,
                "velocity_ff": velocity_ff,
                "current_ff": current_ff,
            },
        )

    def setVelocity(self, velocity: float, current_ff=0):
        velocityCommand = self.motorConverter.convertToMotorCommand(velocity)
        self.socket.sendJSON(
            self.ip,
            PORT_rt,
            {
                "method": "SET",
                "reqTarget": "/m1/setVelocity",
                "velocity": velocityCommand,
                "current_ff": current_ff,
            },
        )

    def setCurrent(self, current: float):
        self.socket.sendJSON(
            self.ip,
            PORT_rt,
            {
                "method": "SET",
                "reqTarget": "/m1/setCurrent",
                "current": current,
            },
        )

    def requestPIDConfig(self):
        self.socket.sendJSON(
            self.ip,
            PORT_srv,
            {
                "method": "GET",
                "reqTarget": "/m1/controller/config",
            },
        )

    def requestRoot(self):
        self.socket.sendJSON(
            self.ip,
            PORT_srv,
            {
                "method": "GET",
                "reqTarget": "/",
            },
        )
    
    def requestRestart(self):
        print("Rebooting...")
        self.socket.sendJSON(
            self.ip, 
            PORT_srv, 
            {
                "method": "SET", 
                "reqTarget": "/", 
                "property": "reboot"
            }        
        )
        


    def setPIDConfig(
        self,
        positionP: float,
        velocityP: float,
        velocityI: float,
        velocityLimit: float,
        limitTolerance: float,
    ):
        velocityLimitCommand = self.motorConverter.convertToMotorCommand(velocityLimit)
        self.socket.sendJSON(
            self.ip,
            PORT_srv,
            {
                "method": "SET",
                "reqTarget": "/m1/controller/config",
                "pos_gain": positionP,
                "vel_gain": velocityP,
                "vel_integrator_gain": velocityI,
                "vel_limit": velocityLimitCommand,
                "vel_limit_tolerance": limitTolerance,
            },
        )

    def getCVP_pt(self):
        tx_messages = struct.pack("<B", 0x1A)
        self.socket.sendBytes(tx_messages)
        try:
            data = self.socket.readBytes()
            feedback = struct.unpack("<fffi", data[1:17])
            return feedback
        except socket.timeout:  # fail after 1 second of no activity
            print("Didn't receive anymore data! [Timeout]")

