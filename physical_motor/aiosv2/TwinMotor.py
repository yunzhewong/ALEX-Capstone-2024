from typing import Callable
from aiosv2 import AiosSocket, ConnectedMotor
from aiosv2.SafeMotorOperation import SafeMotor, SafetyConfiguration


class TwinMotor:
    CONTROL_BOX = "10.10.10.12"
    MOTORS = {"top": "10.10.10.16", "bottom": "10.10.10.17"}
    EXPECTED_IPS = [CONTROL_BOX] + list(MOTORS.values())

    def __init__(self, socket: AiosSocket):
        self.socket = socket
        
        self.socket.assertConnectedAddresses(self.EXPECTED_IPS)
        config = SafetyConfiguration(maximum_current=1, maximum_velocity=100)
        self.topMotor = SafeMotor(self.MOTORS["top"], socket, config)
        self.bottomMotor = SafeMotor(self.MOTORS["bottom"], socket, config)

    def disable(self):
        self.topMotor.disable()
        self.bottomMotor.disable()

    def on_error(self):
        self.topMotor.setCurrent(0)
        self.bottomMotor.setCurrent(0)

def setup_teardown_twin_motor(actions: Callable[[TwinMotor], None]):
    socket = AiosSocket()
    twinMotor = TwinMotor(socket)

    try:
        actions(twinMotor)
    except Exception as e:
        print(e)
        twinMotor.on_error()

    twinMotor.disable()

