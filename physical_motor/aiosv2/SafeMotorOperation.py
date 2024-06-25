import threading
from aiosv2.AiosSocket import AiosSocket
from aiosv2.ConnectedMotor import ConnectedMotor
from aiosv2.constants import ControlMode
from aiosv2.CVP import CVP


class SafetyConfiguration:
    def __init__(self, maximum_current, maximum_velocity):
        self.maximum_current = maximum_current
        self.maximum_velocity = maximum_velocity


class SafeMotor:
    def __init__(self, ip: str, socket: AiosSocket, config: SafetyConfiguration):
        self.raw_motor = ConnectedMotor(ip, socket)
        self.valid = True
        self.config = config
        self.control_mode = None
        self.current_CVP: CVP = CVP(0,0,0)
        self.cvp_lock = threading.Lock()

    def getIP(self):
        return self.raw_motor.ip

    def enable(self):
        self.raw_motor.enable()

    def disable(self):
        self.valid = False
        self.raw_motor.disable()

    def getCVP(self):
        with self.cvp_lock:
            return self.current_CVP

    def setCVP(self, cvp):
        with self.cvp_lock:
            self.current_CVP = cvp
        self.check_within_limits(cvp)

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
        self.raw_motor.setCurrent(current)

    def modeChangeIfNecessary(self, desired_control_mode: ControlMode):
        if (
            self.control_mode is not None
            and self.control_mode.value == desired_control_mode.value
        ):
            return
        self.raw_motor.setControlMode(desired_control_mode)
        self.control_mode = desired_control_mode

    def check_within_limits(self, cvp: CVP):
        abs_velocity = abs(cvp.velocity)
        abs_current = abs(cvp.current)

        if abs_current > self.config.maximum_current:
            raise Exception(f"Exceeded Current Limit: (current: {abs_current:.2f} A, limit: {self.config.maximum_current:.2f} A)")

        if abs_velocity > self.config.maximum_velocity:
            raise Exception(f"Exceeded Velocity Limit: (velocity: {abs_velocity:.2f} rad/s, limit: {self.config.maximum_velocity:.2f} rad/s)")


    def check_operatable(self):
        if not self.valid:
            raise Exception("Motor was already turned off")
