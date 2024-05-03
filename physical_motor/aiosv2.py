from enum import Enum
import json
import math
import socket
import struct
import aios
import threading


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


class SharedInfo:
    def __init__(self, ips):
        self.data = {}
        self.lock = threading.Lock()
        for ip in ips:
            self.data[ip] = {}
        thread = threading.Thread(target=self.readSocket)
        thread.start()

    def readSocket(self):
        while True:
            try:
                data, addr = s.recvfrom(1024)
                ip = addr(0)
                jsonObj = json.loads(data.decode("utf-8"))
                print(jsonObj)
                with self.lock:
                    data[ip][jsonObj["reqTarget"]] = jsonObj
            except Exception as e:
                print(e)


class ConnectedMotor:
    CONVERSION_RATIO = 100000 / math.pi

    def __init__(self, ip: str):
        self.ip = ip

    def sendJSON(self, data: dict, port: int):
        json_str = json.dumps(data)
        s.sendto(str.encode(json_str), (self.ip, port))

    def readJSON(self) -> dict:
        data, _ = s.recvfrom(1024)
        return json.loads(data.decode("utf-8"))

    def getCVP(self) -> CVP:
        data = {
            "method": "GET",
            "reqTarget": "/m1/CVP",
        }
        self.sendJSON(data, PORT_rt)

        json_obj = self.readJSON()
        if json_obj.get("status") != "OK":
            raise Exception("Invalid CVP")

        position = json_obj.get("position") / self.CONVERSION_RATIO
        velocity = json_obj.get("velocity") / self.CONVERSION_RATIO
        current = json_obj.get("current")
        return CVP(current, velocity, position)

    def setControlMode(self, mode: ControlMode):
        self.sendJSON(
            {
                "method": "SET",
                "reqTarget": "/m1/controller/config",
                "control_mode": mode.value,
            },
            PORT_rt,
        )

    def setPosition(self, position: float, velocity_ff=0, current_ff=0):
        self.setControlMode(ControlMode.Position)
        positionCommand = position * self.CONVERSION_RATIO
        self.sendJSON(
            {
                "method": "SET",
                "reqTarget": "/m1/setPosition",
                "reply_enable": False,
                "position": positionCommand,
                "velocity_ff": velocity_ff,
                "current_ff": current_ff,
            },
            PORT_rt,
        )

    def setVelocity(self, velocity: float, current_ff=0):
        self.setControlMode(ControlMode.Velocity)
        velocityCommand = velocity * self.CONVERSION_RATIO
        self.sendJSON(
            {
                "method": "SET",
                "reqTarget": "/m1/setVelocity",
                "reply_enable": False,
                "velocity": velocityCommand,
                "current_ff": current_ff,
            },
            PORT_rt,
        )

    def setCurrent(self, current: float):
        self.setControlMode(ControlMode.Current)
        self.sendJSON(
            {
                "method": "SET",
                "reqTarget": "/m1/setCurrent",
                "reply_enable": False,
                "current": current,
            },
            PORT_rt,
        )

    def getPIDConfig(self):
        data = {
            "method": "GET",
            "reqTarget": "/m1/controller/pid",
        }
        json_str = json.dumps(data)
        print("Send JSON Obj to get PID:", json_str)
        s.sendto(str.encode(json_str), (self.ip, PORT_srv))
        try:
            response, _ = s.recvfrom(1024)
            json_obj = json.loads(response.decode("utf-8"))
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
        s.sendto(tx_messages, (self.ip, PORT_pt))
        try:
            data, address = s.recvfrom(1024)
            feedback = struct.unpack("<fffi", data[1:17])
            return feedback
        except socket.timeout:  # fail after 1 second of no activity
            print("Didn't receive anymore data! [Timeout]")


class ConnectedAddresses:
    def __init__(self, ips):
        self.ips = ips
        # self.sharedInfo = SharedInfo(ips)

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
