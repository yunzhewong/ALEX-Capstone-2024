from aiosv2 import AiosSocket, ConnectedMotor


class TwinMotor:
    CONTROL_BOX = "10.10.10.12"
    MOTORS = {"top": "10.10.10.16", "bottom": "10.10.10.17"}

    def __init__(self, socket: AiosSocket):
        self.socket = socket
        self.topMotor = ConnectedMotor(self.MOTORS["top"], socket)
        self.bottomMotor = ConnectedMotor(self.MOTORS["bottom"], socket)

    def enable(self):
        expectedIPs = self.getExpectedIPs()
        self.socket.assertConnectedAddresses(expectedIPs)

        self.topMotor.enable()
        self.bottomMotor.enable()

    def disable(self):
        self.topMotor.disable()
        self.bottomMotor.disable()

    def getExpectedIPs(self):
        return [self.CONTROL_BOX] + list(self.MOTORS.values())
