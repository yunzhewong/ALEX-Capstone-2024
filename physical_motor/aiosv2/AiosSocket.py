# Create physical and ROS sockets, get addresses of connected devices, send and receive JSON data,
# send and receive bytes
import json
import socket
from aiosv2.constants import PORT_srv

class AiosSocket:
    communicationSocket: socket.socket
    NETWORK = "10.10.10.255"


    def __init__(self):

        newSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        newSocket.settimeout(2)
        newSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.communicationSocket = newSocket


    def readConnectedAddresses(self):
        self.communicationSocket.sendto(
            "Is any AIOS server here?".encode("utf-8"), (self.NETWORK, PORT_srv)
        )

        foundIPs = []

        while True:
            try:
                _, address = self.communicationSocket.recvfrom(1024)
                foundIPs.append(address[0])
            except socket.timeout:
                return foundIPs


    def assertConnectedAddresses(self, expectedIPs):
        print(f"Checking Motor Addresses")
        foundIPs = self.readConnectedAddresses()
        for ip in expectedIPs:
            if ip not in foundIPs:
                raise Exception(f"Missing IP: {ip}")
        print("All Motor IPs connected")
                

    def sendJSON(self, ip: str, port: int, data: dict):
        json_str = json.dumps(data)
        self.communicationSocket.sendto(json_str.encode(), (ip, port))

    def readJSON(self):
        data, addr = self.communicationSocket.recvfrom(1024)

        json_obj = json.loads(data.decode("utf-8"))
        ip = addr[0]
        return (json_obj, ip)

    def sendBytes(self, ip, port, bytesToSend):
        self.communicationSocket.sendto(bytesToSend, (ip, port))

    def readBytes(self) -> bytes:
        data, address = self.communicationSocket.recvfrom(1024)
        return data

    def changeState(self, ip, stateValue):
        self.sendJSON(
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
            self.readJSON()
        except:
            # its fine to not read anything, as we just want to collect it
            pass