from enum import Enum
import json
import math
import socket
import struct
import aios
import threading


# Actuator control mode
class ControlMode(Enum):
    Voltage = 0
    Current = 1
    Velocity = 2
    Position = 3
    Trajectory = 4


PORT_rt = 2333  # Real-time control data port, ie. speed, position, current and other real-time data
PORT_srv = 2334  # Low priority service data port. ie, parameter setting and reading
PORT_pt = 10000  # Passthrough port


class AiosSocket:
    NETWORK = "10.10.10.255"

    s: socket.socket

    def __init__(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s = s

    def get_addresses(self):
        self.s.sendto(
            "Is any AIOS server here?".encode("utf-8"), (self.NETWORK, PORT_srv)
        )
        all_ips = []
        while True:
            try:
                _, address = self.s.recvfrom(1024)
                all_ips.append(address[0])
                found_server = True
            except socket.timeout:  # fail after 1 second of no activity
                if found_server:
                    return ConnectedAddresses(all_ips, self)
                raise Exception("No Addresses Found")

    def sendJSON(self, ip: str, port: int, data: dict):
        json_str = json.dumps(data)
        self.s.sendto(str.encode(json_str), (ip, port))

    def readJSON(self) -> dict:
        data, _ = self.s.recvfrom(1024)
        return json.loads(data.decode("utf-8"))

    def sendBytes(self, ip: str, port: int, bytes: bytes):
        self.s.sendto(bytes, (ip, port))

    def readBytes(self) -> bytes:
        data, address = self.s.recvfrom(1024)
        return data


class CVP:
    def __init__(self, current: float, velocity: float, position: float):
        self.current = current
        self.velocity = velocity
        self.position = position

    def __str__(self):
        return f"Current: {self.current}, Velocity: {self.velocity}, Position: {self.position}"


class ConnectedMotor:
    CONVERSION_RATIO = 100000 / math.pi

    def __init__(self, ip: str, socket: AiosSocket):
        self.ip = ip
        self.socket = socket

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

        json_obj = self.socket.readJSON()
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


class ConnectedAddresses:
    def __init__(self, ips, socket: AiosSocket):
        self.ips = ips
        self.socket = socket

    def enable(self):
        for ip in self.ips:
            aios.enable(ip, 1)

    def disable(self):
        for ip in self.ips:
            aios.disable(ip, 1)

    def getConnectedMotors(self) -> list[ConnectedMotor]:
        connectedMotors = []
        for ip in self.ips:
            root = aios.getRoot(ip)
            is_control_box = root.get("deviceType", None) == "ctrlbox"
            if not is_control_box:
                connectedMotors.append(ConnectedMotor(ip, self.socket))

        return connectedMotors
