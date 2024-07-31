from enum import Enum
import threading
from aiosv2.constants import ControlMode, Converter
from aiosv2.ConnectedMotor import ConnectedMotor
from aiosv2.CVP import CVP


class ExceedRange(Enum):
    BelowLowerLimit = "Below Lower Limit"
    AboveUpperLimit = "Above Upper Limit"


class SafetyValueRange:
    def __init__(self, low: float, high: float):
        self.low = low
        self.high = high

    def get_value_limit(self, value):
        if value < self.low:
            return ExceedRange.BelowLowerLimit, self.low

        if value > self.high:
            return ExceedRange.AboveUpperLimit, self.high

        return None


class SafetyLimit:
    def __init__(
        self, name: str, soft_range: SafetyValueRange, hard_range: SafetyValueRange
    ):
        self.name = name
        self.soft_range = soft_range
        self.hard_range = hard_range

    def check_value(self, value):
        hard_exceed = self.hard_range.get_value_limit(value)

        if hard_exceed is not None:
            raise Exception(
                f"Exceeded {self.name} Hard Limit - ({self.name} value: {value}, expected range: [{self.hard_range.low}, {self.hard_range.high}])"
            )

        return self.soft_range.get_value_limit(value)


class SafetyConfiguration:
    def __init__(
        self,
        current_limit: SafetyLimit,
        velocity_limit: SafetyLimit,
        position_limit: SafetyLimit,
    ):
        self.current_limit: SafetyLimit = current_limit
        self.velocity_limit: SafetyLimit = velocity_limit
        self.position_limit: SafetyLimit = position_limit

    def check_within_limits(self, cvp: CVP):
        self.position_limit.check_value(cvp.position)
        self.velocity_limit.check_value(cvp.velocity)
        self.current_limit.check_value(cvp.current)

    def override_if_unsafe(self, cvp: CVP | None, controlMode: ControlMode, value: float):
        if cvp is None:
            return value
        position_constraint = self.position_limit.check_value(cvp.position)

        if position_constraint is not None:
            limit_type, limit_value = position_constraint
            if controlMode == ControlMode.Position:
                return limit_value
            else:
                # force control to point in other direction of limit
                if limit_type == ExceedRange.AboveUpperLimit:
                    return min(value, -5)
                else:
                    return max(value, 5)

        velocity_constraint = self.velocity_limit.check_value(cvp.velocity)

        if velocity_constraint is not None:
            limit_type, limit_value = velocity_constraint
            if controlMode == ControlMode.Position:
                return cvp.position
            if controlMode == ControlMode.Velocity:
                if limit_type == ExceedRange.AboveUpperLimit:
                    return min(value, limit_value)
                else:
                    return max(value, limit_value)
            if controlMode == ControlMode.Current:
                if limit_type == ExceedRange.AboveUpperLimit:
                    return min(value, 0)
                else:
                    return max(value, 0)

        current_constraint = self.current_limit.check_value(cvp.current)

        if current_constraint is not None:
            limit_type, limit_value = current_constraint
            if controlMode == ControlMode.Position:
                return cvp.position
            if controlMode == ControlMode.Velocity:
                if limit_type == ExceedRange.AboveUpperLimit:
                    return min(value, 0)
                else:
                    return max(value, 0)
            if controlMode == ControlMode.Current:
                if limit_type == ExceedRange.AboveUpperLimit:
                    return min(value, limit_value)
                else:
                    return max(value, limit_value)

        return value


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

        control_type = ControlMode.Current
        self.control_mode: ControlMode = control_type
        self.raw_motor.setControlMode(control_type)
        self.raw_motor.setInputMode(control_type)

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
        self.raw_motor.setCurrent(0)
        self.valid = False
        self.raw_motor.disable()

    def getCVP(self):
        if self.current_CVP is None:
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

        cvp = self.getCVP()
        override_value = self.config.override_if_unsafe(
            cvp, ControlMode.Position, position
        )
        self.raw_motor.setPosition(override_value)

    def setVelocity(self, velocity: float):
        with self.config_lock:
            if not self.config_ready:
                return
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Velocity)

        cvp = self.getCVP()
        override_value = self.config.override_if_unsafe(
            cvp, ControlMode.Velocity, velocity
        )
        self.raw_motor.setVelocity(override_value)

    def setCurrent(self, current: float):
        with self.config_lock:
            if not self.config_ready:
                return
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Current)
        cvp = self.getCVP()
        override_value = self.config.override_if_unsafe(
            cvp, ControlMode.Current, current
        )
        self.raw_motor.setCurrent(override_value)

    def modeChangeIfNecessary(self, desired_control_mode: ControlMode):
        if self.control_mode.value == desired_control_mode.value:
            return
        self.raw_motor.setControlMode(desired_control_mode)
        self.control_mode = desired_control_mode

    def check_operatable(self):
        if not self.valid:
            raise Exception("Motor was already turned off")
