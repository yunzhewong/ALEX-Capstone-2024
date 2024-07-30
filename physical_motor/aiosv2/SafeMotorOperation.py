import threading
import time
from aiosv2.AiosSocket import AiosSocket
from aiosv2.constants import ControlMode, Converter
from aiosv2.ConnectedMotor import ConnectedMotor
from aiosv2.CVP import CVP


class SafetyConfiguration:
    def __init__(
        self,
        margin,
        maximum_current,
        maximum_velocity,
        minimum_position,
        maximum_position,
    ):
        self.margin = margin
        self.maximum_current = maximum_current * (1 - margin)
        self.maximum_velocity = maximum_velocity * (1 - margin)
        position_range = maximum_position - minimum_position
        self.minimum_position = minimum_position + (margin * position_range)
        self.maximum_position = maximum_position - (margin * position_range)

    def check_within_limits(self, cvp: CVP):
        pos = cvp.position
        abs_velocity = abs(cvp.velocity)
        abs_current = abs(cvp.current)

        if abs_current >= self.maximum_current:
            raise Exception(
                f"Within Current Limit Margin: (current: {abs_current:.2f} A, limit: {self.maximum_current:.2f} A)"
            )

        if abs_velocity >= self.maximum_velocity:
            raise Exception(
                f"Within Velocity Limit Margin: (velocity: {abs_velocity:.2f} rad/s, limit: {self.maximum_velocity:.2f} rad/s)"
            )

        if pos <= self.minimum_position:
            raise Exception(
                f"Within Minimum Position Margin: (position: {pos:.2f} rad, limit: {self.minimum_position:.2f} rad)"
            )

        if pos >= self.maximum_position:
            raise Exception(
                f"Within Maximum Position Margin: (position: {pos:.2f} rad, limit: {self.maximum_position:.2f} rad)"
            )


class SafeMotor:
    def __init__(
        self,
        ip: str,
        socket,
        config: SafetyConfiguration,
        motorConverter: Converter,
    ):
        self.raw_motor = ConnectedMotor(ip, socket, motorConverter)
        self.valid: bool = True
        self.config = config
        self.control_mode: ControlMode = ControlMode.Position
        self.raw_motor.setControlMode(ControlMode.Position)

        self.current_CVP: CVP | None = None
        self.cvp_lock = threading.Lock()
        self.encoder_ready = False
        self.encoder_lock = threading.Lock()
        self.config_ready = True
        self.config_lock = threading.Lock()

    def getIP(self):
        return self.raw_motor.ip

    def enable(self):
        self.raw_motor.enable()

    def disable(self):
        self.valid = False
        self.raw_motor.disable()

    def getCVP(self):
        if self.current_CVP is None:
            self.setCurrent(0)
            return None
        with self.cvp_lock:
            return self.current_CVP

    def setCVP(self, cvp):
        with self.cvp_lock:
            self.current_CVP = cvp
        self.config.check_within_limits(cvp)

    def encoderIsReady(self):
        with self.encoder_lock:
            return self.encoder_ready

    def requestEncoderReady(self):
        self.raw_motor.requestEncoderCheck()

    def confirmEncoderReady(self):
        with self.encoder_lock:
            self.encoder_ready = True

    def setPosition(self, position: float):
        with self.config_lock:
            if not self.config_ready:
                return
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Position)
        self.raw_motor.setPosition(position)

    def setVelocity(self, velocity: float):
        with self.config_lock:
            if not self.config_ready:
                return
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Velocity)
        self.raw_motor.setVelocity(velocity)

    def setCurrent(self, current: float):
        with self.config_lock:
            if not self.config_ready:
                return
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Current)
        self.raw_motor.setCurrent(current)

    def modeChangeIfNecessary(self, desired_control_mode: ControlMode):
        if self.control_mode.value == desired_control_mode.value:
            return
        self.raw_motor.setControlMode(desired_control_mode)
        self.control_mode = desired_control_mode

    def check_operatable(self):
        if not self.valid:
            raise Exception("Motor was already turned off")
