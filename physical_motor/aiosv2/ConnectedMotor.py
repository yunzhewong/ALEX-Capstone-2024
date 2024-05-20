import math
import socket
import struct
from aiosv2.constants import ControlMode, AxisState, PORT_rt, PORT_pt, PORT_srv
from aiosv2.CVP import CVP
from aiosv2.AiosSocket import AiosSocket


# Represents a motor connected to the system
# Has methods to request and get the Current, Velocity and Position of the motor
class ConnectedMotor:
    CONVERSION_RATIO = 100000 / math.pi

    def __init__(self, ip: str, socket: AiosSocket):
        self.ip = ip
        self.socket = socket

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

    def getCVP(self) -> CVP:
        data = {
            "method": "GET",
            "reqTarget": "/m1/CVP",
        }
        self.socket.sendJSON(self.ip, PORT_rt, data)

        json_obj, _ = self.socket.readJSON()
        if json_obj.get("status") != "OK":
            raise Exception("Invalid CVP")

        position = json_obj.get("position") / self.CONVERSION_RATIO
        velocity = json_obj.get("velocity") / self.CONVERSION_RATIO
        current = json_obj.get("current")
        return CVP(current, velocity, position)

    def setControlMode(self, mode: ControlMode):
        self.socket.sendJSON(
            self.ip,
            PORT_rt,
            {
                "method": "SET",
                "reqTarget": "/m1/controller/config",
                "control_mode": mode.value,
            },
        )

    def setPosition(self, position: float, velocity_ff=0, current_ff=0):
        self.setControlMode(ControlMode.Position)
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
        self.setControlMode(ControlMode.Velocity)
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
        self.setControlMode(ControlMode.Current)
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
                "reqTarget": "/m1/controller/pid",
            },
        )
        try:
            json_obj = self.socket.readJSON()
            if json_obj.get("status") == "OK":
                return {
                    "kp": json_obj.get("kp"),
                    "ki": json_obj.get("ki"),
                    "kd": json_obj.get("kd"),
                }
            else:
                print("Failed to get PID config:", json_obj.get("error"))
                return None
        except socket.timeout:
            print("Network Timeout, no response received.")
            return None
        except Exception as e:
            print(f"Error fetching PID config: {str(e)}")
            return None

    def getCVP_pt(self):
        tx_messages = struct.pack("<B", 0x1A)
        self.socket.sendBytes(tx_messages)
        try:
            data = self.socket.readBytes()
            feedback = struct.unpack("<fffi", data[1:17])
            return feedback
        except socket.timeout:  # fail after 1 second of no activity
            print("Didn't receive anymore data! [Timeout]")
