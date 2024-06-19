from AiosSocket import AiosSocket
from ConnectedMotor import ConnectedMotor

class SafetyConfiguration:
    def __init__(self, maximum_current, maximum_velocity):
        self.maximum_current = maximum_current
        self.maximum_velocity = maximum_velocity
    

class SafeMotor:
    def __init__(self, ip: str, socket: AiosSocket, config: SafetyConfiguration):
        self.raw_motor = ConnectedMotor(ip, socket)
        self.config = config
    

