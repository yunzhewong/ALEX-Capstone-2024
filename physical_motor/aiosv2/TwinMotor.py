from typing import Callable
from aiosv2 import AiosSocket, ConnectedMotor
from SafeMotorOperation import SafeMotor, SafetyConfiguration


class TwinMotor:
    CONTROL_BOX = "10.10.10.12"
    MOTORS = {"top": "10.10.10.16", "bottom": "10.10.10.17"}
    EXPECTED_IPS = [CONTROL_BOX] + list(MOTORS.values())

    def __init__(self, socket: AiosSocket):
        self.socket = socket
        
        config = SafetyConfiguration(maximum_current=3, maximum_velocity=100)
        self.topMotor = SafeMotor(self.MOTORS["top"], socket, config)
        self.bottomMotor = SafeMotor(self.MOTORS["bottom"], socket, config)

    def enable(self):
        self.socket.assertConnectedAddresses(self.EXPECTED_IPS)

        self.topMotor.enable()
        self.bottomMotor.enable()

    def disable(self):
        self.topMotor.disable()
        self.bottomMotor.disable()

def setup_teardown_twin_motor(actions: Callable[[TwinMotor]]):
    socket = AiosSocket()
    twinMotor = TwinMotor(socket)
    twinMotor.enable()

    actions(twinMotor)

    twinMotor.disable()
