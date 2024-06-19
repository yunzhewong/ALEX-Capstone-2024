from AiosSocket import AiosSocket
from ConnectedMotor import ConnectedMotor
from constants import ControlMode
from CVP import CVP

class SafetyConfiguration:
    def __init__(self, maximum_current, maximum_velocity):
        self.maximum_current = maximum_current
        self.maximum_velocity = maximum_velocity
    

class SafeMotor:
    def __init__(self, ip: str, socket: AiosSocket, config: SafetyConfiguration):
        self.raw_motor = ConnectedMotor(ip, socket)
        self.raw_motor.enable()
        self.valid = True
        self.config = config
        self.control_mode = ControlMode.Current
        self.last_checked_CVP: CVP | None = None
        
    def disable(self):
        self.valid = False
        self.raw_motor.disable()

    def getMeasuredCVP(self, cached=True):
        if self.last_checked_CVP is None or not cached:
            return self.raw_motor.getCVP()
        return self.last_checked_CVP
    
    def setPosition(self, position: float):
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Position)
        self.raw_motor.setPosition(position)

    def setVelocity(self, velocity: float):
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Velocity)
        self.raw_motor.setVelocity(velocity)

    def setCurrent(self, current: float):
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Current)
        self.setCurrent(current)

    def modeChangeIfNecessary(self, desired_control_mode: ControlMode):
        if self.control_mode == desired_control_mode:
            return
        
        self.raw_motor.setControlMode(desired_control_mode)        
        self.control_mode = desired_control_mode

    def check_within_limits(self):
        cvp = self.raw_motor.getCVP()

        within_velocity_limits = abs(cvp.velocity) < self.config.maximum_velocity
        within_current_limits = abs(cvp.current) < self.config.maximum_current

        if not within_current_limits or not within_velocity_limits:
            raise Exception("Driving outside of the preset limits")
        
        self.last_checked_CVP = cvp

    def check_operatable(self):
        if not self.valid:
            raise Exception("Motor was already turned off")
    
    
    

    
