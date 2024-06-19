from aiosv2 import AiosSocket, ConnectedMotor


class TwinMotor:
    CONTROL_BOX = "10.10.10.12"
    MOTORS = {"top": "10.10.10.16", "bottom": "10.10.10.17"}
    EXPECTED_IPS = [CONTROL_BOX] + list(MOTORS.values())

    def __init__(self, socket: AiosSocket):
        self.socket = socket
        self.topMotor = ConnectedMotor(self.MOTORS["top"], socket)
        self.bottomMotor = ConnectedMotor(self.MOTORS["bottom"], socket)

    def enable(self):
        self.socket.assertConnectedAddresses(self.EXPECTED_IPS)

        self.topMotor.enable()
        self.bottomMotor.enable()

    def disable(self):
        self.topMotor.disable()
        self.bottomMotor.disable()

