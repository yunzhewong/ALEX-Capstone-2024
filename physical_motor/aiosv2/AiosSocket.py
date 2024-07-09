# Create physical and ROS sockets, get addresses of connected devices, send and receive JSON data,
# send and receive bytes
import json
import socket
from aiosv2.constants import PORT_srv

class PhysicalSocket:
    NETWORK = "10.10.10.255"

    def __init__(self):
        physicalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        physicalSocket.settimeout(2)
        physicalSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket = physicalSocket

    def assertConnectedAddresses(self, expectedIPs):
        self.socket.sendto(
            "Is any AIOS server here?".encode("utf-8"), (self.NETWORK, PORT_srv)
        )

        foundIPs = []
        while True:
            try:
                _, address = self.socket.recvfrom(1024)
                foundIPs.append(address[0])
            except socket.timeout:  # fail after 1 second of no activity
                for ip in expectedIPs:
                    if ip not in foundIPs:
                        raise Exception(f"Missing IP: {ip}")
                return

    def sendJSON(self, ip, port, data):
        json_str = json.dumps(data)
        self.socket.sendto(json_str.encode(), (ip, port))

    def readJSON(self):
        data, addr = self.socket.recvfrom(1024)

        json_obj = json.loads(data.decode("utf-8"))
        ip = addr[0]
        return (json_obj, ip)

    def sendBytes(self, ip, port, bytesToSend):
        self.socket.sendto(bytesToSend, (ip, port))

    def readBytes(self):
        data, address = self.socket.recvfrom(1024)
        return data

class AiosSocket:
    physicalSocket: PhysicalSocket

    def __init__(self):
        print(f"Physical Socket Active")

        self.physicalSocket = PhysicalSocket()

    def assertConnectedAddresses(self, expectedIPs):
        self.physicalSocket.assertConnectedAddresses(expectedIPs)

    def sendJSON(self, ip: str, port: int, data: dict):
        self.physicalSocket.sendJSON(ip, port, data)

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