from enum import Enum
import json
import math
import socket
import aios


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

PORT_rt = 2333  # Real-time control data port, ie. speed, position, current and other real-time data
PORT_srv = 2334  # Low priority service data port. ie, parameter setting and reading
PORT_pt = 10000  # Passthrough port

NETWORK = "10.10.10.255"


# Actuator control mode
class ControlMode(Enum):
    Voltage = 0
    Current = 1
    Velocity = 2
    Position = 3
    Trajectory = 4


class CVP:
    def __init__(self, current: float, velocity: float, position: float):
        self.current = current
        self.velocity = velocity
        self.position = position

    def __str__(self):
        return f"Current: {self.current}, Velocity: {self.velocity}, Position: {self.position}"


class ConnectedMotor:
    CONVERSION_RATIO = 100000 / math.pi

    def __init__(self, ip):
        self.ip = ip

    def getCVP(self) -> CVP:
        data = {
            "method": "GET",
            "reqTarget": "/m1/CVP",
        }
        json_str = json.dumps(data)
        s.sendto(str.encode(json_str), (self.ip, PORT_rt))
        data, _ = s.recvfrom(1024)
        json_obj = json.loads(data.decode("utf-8"))
        if json_obj.get("status") != "OK":
            raise Exception("Invalid CVP")

        position = json_obj.get("position") / self.CONVERSION_RATIO
        velocity = json_obj.get("velocity") / self.CONVERSION_RATIO
        current = json_obj.get("current")
        return CVP(current, velocity, position)

    def setControlMode(self, mode: ControlMode):
        data = {
            "method": "SET",
            "reqTarget": "/m1/controller/config",
            "control_mode": mode.value,
        }
        json_str = json.dumps(data)
        s.sendto(str.encode(json_str), (self.ip, PORT_srv))
        s.recvfrom(1024)

    def setPosition(self, position: float, velocity_ff=0, current_ff=0):
        self.setControlMode(ControlMode.Position)
        positionCommand = position * self.CONVERSION_RATIO
        data = {
            "method": "SET",
            "reqTarget": "/m1/setPosition",
            "reply_enable": False,
            "position": positionCommand,
            "velocity_ff": velocity_ff,
            "current_ff": current_ff,
        }
        json_str = json.dumps(data)
        print("Send JSON Obj:", json_str)
        s.sendto(str.encode(json_str), (self.ip, PORT_rt))

    def setVelocity(self, velocity: float, current_ff=0):
        self.setControlMode(ControlMode.Velocity)
        velocityCommand = velocity * self.CONVERSION_RATIO
        data = {
            "method": "SET",
            "reqTarget": "/m1/setVelocity",
            "reply_enable": False,
            "velocity": velocityCommand,
            "current_ff": current_ff,
        }
        json_str = json.dumps(data)
        print("Send JSON Obj:", json_str)
        s.sendto(str.encode(json_str), (self.ip, PORT_rt))

    def setCurrent(self, current: float):
        data = {
            "method": "SET",
            "reqTarget": "/m1/setCurrent",
            "reply_enable": False,
            "current": current,
        }
        json_str = json.dumps(data)
        print("Send JSON Obj:", json_str)
        s.sendto(str.encode(json_str), (self.ip, PORT_rt))


class ConnectedAddresses:
    def __init__(self, ips):
        self.ips = ips

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
                connectedMotors.append(ConnectedMotor(ip))

        return connectedMotors


def get_addresses():
    s.sendto("Is any AIOS server here?".encode("utf-8"), (NETWORK, PORT_srv))
    all_ips = []
    while True:
        try:
            _, address = s.recvfrom(1024)
            all_ips.append(address[0])
            found_server = True
        except socket.timeout:  # fail after 1 second of no activity
            if found_server:
                return ConnectedAddresses(all_ips)
            raise Exception("No Addresses Found")
