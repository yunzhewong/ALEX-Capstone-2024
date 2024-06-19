import math
import socket
import struct
import time
from typing import Optional
from aiosv2.constants import ControlMode, AxisState, PORT_rt, PORT_pt, PORT_srv
from aiosv2.CVP import CVP
from aiosv2.AiosSocket import AiosSocket


class PIDConfig:
    def __init__(self, response):
        self.positionP = response.get("pos_gain")
        self.velocityP = response.get("vel_gain")
        self.velocityI = response.get("vel_integrator_gain")
        self.velocityLimit = response.get("vel_limit")
        self.limitTolerance = response.get("vel_limit_tolerance")

    def __str__(self):
        return f"Position Gain: {self.positionP}, Velocity Gain: {self.velocityP}, Velocity Int: {self. velocityI}"


# Represents a motor connected to the system
# Has methods to request and get the Current, Velocity and Position of the motor
class ConnectedMotor:
    CONVERSION_RATIO = 100000 / math.pi

    def __init__(self, ip: str, socket: AiosSocket):
        self.ip = ip
        self.socket = socket
        self.controlMode = ControlMode.Current

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

    def getCVP(self) -> Optional[CVP]:
        self.socket.sendJSON(self.ip, PORT_rt, {
            "method": "GET",
            "reqTarget": "/m1/CVP",
        })

        response = self.socket.readJSON()

        if response is None:
            raise Exception("Could not read CVP")
        json_obj, _ = response

        readStatus = json_obj.get("status")
        readPosition = json_obj.get("position")
        readVelocity = json_obj.get("velocity")
        readCurrent = json_obj.get("current")

        validStatus = readStatus == "OK"
        validPosition = readPosition is not None
        validVelocity = readVelocity is not None
        validCurrent = readCurrent is not None

        if not validStatus or not validPosition or not validVelocity or not validCurrent:
            raise Exception("Invalid CVP")

        position = readPosition / self.CONVERSION_RATIO
        velocity = readVelocity / self.CONVERSION_RATIO
        current = readCurrent
        return CVP(current, velocity, position)

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
        positionCommand = position * self.CONVERSION_RATIO
        self.socket.sendJSON(
            self.ip,
            PORT_rt,
            {
                "method": "SET",
                "reqTarget": "/m1/setPosition",
                "reply_enable": False,
                "position": positionCommand,
                "velocity_ff": velocity_ff,
                "current_ff": current_ff,
            },
        )

    def setVelocity(self, velocity: float, current_ff=0):
        velocityCommand = velocity * self.CONVERSION_RATIO
        self.socket.sendJSON(
            self.ip,
            PORT_rt,
            {
                "method": "SET",
                "reqTarget": "/m1/setVelocity",
                "reply_enable": False,
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
                "reply_enable": False,
                "current": current,
            },
        )

    def getPIDConfig(self):
        self.socket.sendJSON(
            self.ip,
            PORT_srv,
            {
                "method": "GET",
                "reqTarget": "/m1/controller/config",
            },
        )

        response, _ = self.socket.readJSON()

        return PIDConfig(response)

    def getCVP_pt(self):
        tx_messages = struct.pack("<B", 0x1A)
        self.socket.sendBytes(tx_messages)
        try:
            data = self.socket.readBytes()
            feedback = struct.unpack("<fffi", data[1:17])
            return feedback
        except socket.timeout:  # fail after 1 second of no activity
            print("Didn't receive anymore data! [Timeout]")
