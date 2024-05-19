import socket
import physical_motor.aiosv2.aios as aios


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

PORT_rt = 2333  # Real-time control data port, ie. speed, position, current and other real-time data
PORT_srv = 2334  # Low priority service data port. ie, parameter setting and reading
PORT_pt = 10000  # Passthrough port

NETWORK = "10.10.10.255"


class ConnectedAddresses:
    def __init__(self, ips):
        self.ips = ips

    def enable(self):
        for ip in self.ips:
            aios.enable(ip, 1)

    def disable(self):
        for ip in self.ips:
            aios.disable(ip, 1)

    def get_motor_ips(self):
        motor_ips = []
        for ip in self.ips:
            root = aios.getRoot(ip)
            is_control_box = root.get("deviceType", None) == "ctrlbox"
            if not is_control_box:
                motor_ips.append(ip)

        return motor_ips


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


def separate_ips(ips):

    print()
