from AiosSocket import AiosSocket
from ConnectedMotor import ConnectedMotor
from constants import ControlMode

class SafetyConfiguration:
    def __init__(self, maximum_current, maximum_velocity):
        self.maximum_current = maximum_current
        self.maximum_velocity = maximum_velocity
    

class SafeMotor:
    def __init__(self, ip: str, socket: AiosSocket, config: SafetyConfiguration):
        self.raw_motor = ConnectedMotor(ip, socket)
        self.config = config
        self.controlMode = ControlMode.Current
    
    def setPosition(self, position: float):
        self.modeChangeIfNecessary(ControlMode.Position)
        self.raw_motor.setPosition(position)

    def setVelocity(self, velocity: float):
        self.modeChangeIfNecessary(ControlMode.Velocity)
        self.raw_motor.setVelocity(velocity)

    def setCurrent(self, current: float):
        self.modeChangeIfNecessary(ControlMode.Current)
        self.setCurrent(current)
    
    def modeChangeIfNecessary(self, desiredControlMode: ControlMode):
        if self.controlMode == desiredControlMode:
            return
        
        self.raw_motor.setControlMode(desiredControlMode)        
        self.controlMode = desiredControlMode

    

    
