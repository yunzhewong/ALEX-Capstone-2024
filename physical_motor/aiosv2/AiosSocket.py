# Create physical and ROS sockets, get addresses of connected devices, send and receive JSON data,
# send and receive bytes
import json
import socket
from aiosv2.constants import PORT_srv
from aiosv2.serverConstants import ROS_HOST, ROS_PORT


PHYSICAL_ACTIVE = True
ROS_ACTIVE = False


class PhysicalSocket:
    NETWORK = "10.10.10.255"

    def __init__(self, connected):
        self.connected = connected

        if not self.connected:
            return

        physicalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        physicalSocket.settimeout(2)
        physicalSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket = physicalSocket

    def assertConnectedAddresses(self, expectedIPs):
        if not self.connected:
            return

        self.socket.sendto(
            "Is any AIOS server here?".encode("utf-8"), (self.NETWORK, PORT_srv)
        )

        foundIPs = []
        while True:
            try:
                _, address = self.socket.recvfrom(1024)
                foundIPs.append(address[0])
            except socket.timeout:  # fail after 1 second of no activity
                print(foundIPs)
                for ip in expectedIPs:
                    if ip not in foundIPs:
                        raise Exception(f"Missing IP: {ip}")
                return

    def sendJSON(self, ip, port, data):
        if not self.connected:
            return

        json_str = json.dumps(data)
        self.socket.sendto(json_str.encode(), (ip, port))

    def readJSON(self):
        if not self.connected:
            return None

        data, addr = self.socket.recvfrom(1024)

        json_obj = json.loads(data.decode("utf-8"))
        ip = addr[0]
        return (json_obj, ip)

    def sendBytes(self, ip, port, bytesToSend):
        if not self.connected:
            return
        self.socket.sendto(bytesToSend, (ip, port))

    def readBytes(self):
        data, address = self.socket.recvfrom(1024)
        return data


class RosSocket:
    def __init__(self, connected):
        self.connected = connected

        if not self.connected:
            return

        rosSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rosSocket.connect((ROS_HOST, ROS_PORT))
        print("Connected")
        rosSocket.send("".encode())
        self.socket = rosSocket

    def sendJSON(self, ip, port, data):
        if not self.connected:
            return

        obj = {"IP": ip, "commandJSON": data}
        json_str = json.dumps(obj)
        self.socket.send(json_str.encode())


class AiosSocket:
    physicalSocket: PhysicalSocket
    rosSocket: RosSocket

    def __init__(self):
        self.physicalSocket = PhysicalSocket(PHYSICAL_ACTIVE)
        self.rosSocket = RosSocket(ROS_ACTIVE)

    def assertConnectedAddresses(self, expectedIPs):
        self.physicalSocket.assertConnectedAddresses(expectedIPs)

    def sendJSON(self, ip: str, port: int, data: dict):
        self.physicalSocket.sendJSON(ip, port, data)
        self.rosSocket.sendJSON(ip, port, data)

    def readJSON(self):
        return self.physicalSocket.readJSON()

    def sendBytes(self, ip: str, port: int, bytesToSend: bytes):
        self.physicalSocket.sendBytes(ip, port, bytesToSend)

    def readBytes(self) -> bytes:
        return self.physicalSocket.readBytes()

    def changeState(self, ip, stateValue):
        self.physicalSocket.sendJSON(
            ip,
            PORT_srv,
            {
                "method": "SET",
                "reqTarget": "/m1/requested_state",
                "property": stateValue,
                "reply_enable": False,
            },
        )
        try:
            self.physicalSocket.readJSON()
        except:
            # its fine to not read anything, as we just want to collect it
            pass