# Create physical and ROS sockets, get addresses of connected devices, send and receive JSON data,
# send and receive bytes
import json
import socket
from typing import Optional
from aiosv2.constants import PORT_srv
from aiosv2.serverConstants import ROS_HOST, ROS_PORT


class AiosSocket:
    NETWORK = "10.10.10.255"
    PHYSICAL_ACTIVE = True
    ROS_ACTIVE = False

    physicalSocket: Optional[socket.socket]

    rosSocket: Optional[socket.socket]

    def __init__(self):
        self.physicalSocket = None
        self.rosSocket = None
        if self.PHYSICAL_ACTIVE:
            self.createPhysicalSocket()

        if self.ROS_ACTIVE:
            self.createROSSocket()

    def createPhysicalSocket(self):
        physicalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        physicalSocket.settimeout(2)
        physicalSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.physicalSocket = physicalSocket

    def createROSSocket(self):
        rosSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rosSocket.connect((ROS_HOST, ROS_PORT))
        self.rosSocket = rosSocket

    def assertConnectedAddresses(self, expectedIPs):
        if self.physicalSocket is None:
            return

        self.physicalSocket.sendto(
            "Is any AIOS server here?".encode("utf-8"), (self.NETWORK, PORT_srv)
        )
        foundIPs = []
        while True:
            try:
                _, address = self.physicalSocket.recvfrom(1024)
                foundIPs.append(address[0])
            except socket.timeout:  # fail after 1 second of no activity
                for ip in expectedIPs:
                    if ip not in foundIPs:
                        raise Exception(f"Missing IP: {ip}")
                return

    def sendJSON(self, ip: str, port: int, data: dict):
        if self.physicalSocket is not None:
            json_str = json.dumps(data)
            self.physicalSocket.sendto(json_str.encode(), (ip, port))

        if self.rosSocket is not None:
            obj = {"IP": ip, "commandJSON": data}
            json_str = json.dumps(obj)
            self.rosSocket.send(json_str.encode())

    def readJSON(self):
        if self.physicalSocket is None:
            return None

        data, addr = self.physicalSocket.recvfrom(1024)

        json_obj = json.loads(data.decode("utf-8"))
        ip = addr[0]
        return (json_obj, ip)

    def sendBytes(self, ip: str, port: int, bytes: bytes):
        if self.physicalSocket is None:
            raise Exception("Not physically connected")

        self.physicalSocket.sendto(bytes, (ip, port))

    def readBytes(self) -> bytes:
        if self.physicalSocket is None:
            raise Exception("Not physically connected")
        data, address = self.physicalSocket.recvfrom(1024)
        return data
