from enum import Enum
import threading
from aiosv2.AiosSocket import AiosSocket
from aiosv2.constants import ControlMode, Converter
from aiosv2.ConnectedMotor import ConnectedMotor
from aiosv2.CVP import CVP


class ExceedRange(Enum):
    BelowLowerLimit = "Below Lower Limit"
    AboveUpperLimit = "Above Upper Limit"


class SafetyValueRange:
    def __init__(self, configuration: dict):
        self.low = configuration["low"]
        self.high = configuration["high"]

    def get_value_limit(self, value):
        if value < self.low:
            return ExceedRange.BelowLowerLimit, self.low

        if value > self.high:
            return ExceedRange.AboveUpperLimit, self.high

        return None


class SafetyLimit:
    def __init__(self, name: str, configuration: dict):
        self.name = name
        self.soft_range = SafetyValueRange(configuration["soft"])
        self.hard_range = SafetyValueRange(configuration["hard"])

    def check_value(self, value):
        hard_exceed = self.hard_range.get_value_limit(value)

        if hard_exceed is not None:
            raise Exception(
                f"Exceeded {self.name} Hard Limit - ({self.name} value: {value}, expected range: [{self.hard_range.low}, {self.hard_range.high}])"
            )

        return self.soft_range.get_value_limit(value)


class SafetyConfiguration:
    def __init__(self, safetyConfiguration: dict):
        self.current_limit = SafetyLimit("Current", safetyConfiguration["current"])
        self.velocity_limit = SafetyLimit("Velocity", safetyConfiguration["velocity"])
        self.position_limit = SafetyLimit("Position", safetyConfiguration["position"])

    def check_within_limits(self, cvp: CVP):
        self.position_limit.check_value(cvp.position)
        self.velocity_limit.check_value(cvp.velocity)
        self.current_limit.check_value(cvp.current)

    def get_velocity_limit_settings(self):
        soft_low = self.velocity_limit.soft_range.low
        soft_high = self.velocity_limit.soft_range.high
        hard_low = self.velocity_limit.hard_range.low
        hard_high = self.velocity_limit.hard_range.high

        if soft_low != soft_high * -1:
            print("Non-Symmetrical Soft Limits - using lower value")

        if hard_low != hard_high * -1:
            print("Non-Symmetrical Hard Limits - using lower value")

        limit_soft = min(abs(soft_low), abs(soft_high))
        limit_hard = min(abs(hard_low), abs(hard_high))

        tolerance = limit_hard / limit_soft
        return limit_soft, tolerance

    def override_if_unsafe(
        self, cvp: CVP | None, controlMode: ControlMode, value: float
    ):
        if cvp is None:
            return value
        position_constraint = self.position_limit.check_value(cvp.position)

        if position_constraint is not None:
            limit_type, limit_value = position_constraint
            print(f"Exceeded Soft Position Limit: {limit_type}")
            if controlMode == ControlMode.Position:
                return limit_value
            else:
                # force control to point in other direction of limit
                if limit_type == ExceedRange.AboveUpperLimit:
                    return min(value, 0)
                else:
                    return max(value, 0)

        velocity_constraint = self.velocity_limit.check_value(cvp.velocity)

        if velocity_constraint is not None:
            limit_type, limit_value = velocity_constraint
            print(f"Exceeded Soft Velocity Limit: {limit_type}")
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
            print(f"Exceeded Soft Current Limit: {limit_type}")
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
        configuration: dict,
        socket: AiosSocket,
        motorConverter: Converter,
        calibrationAdjustment: float,
        name: str = None  # Added name parameter
    ):
        self.name = name if name else configuration.get("name", "Unnamed Motor")  # Set the name attribute
        self.configuration = configuration
        self.raw_motor = ConnectedMotor(configuration["ip"], socket, motorConverter)
        self.valid: bool = True
        self.safetyConfiguration = SafetyConfiguration(configuration["safetyConfiguration"])
        self.current_CVP: CVP | None = None
        self.cvp_lock = threading.Lock()
        self.encoder_ready = False
        self.encoder_lock = threading.Lock()
        self.config_ready = True
        self.config_lock = threading.Lock()
        self.calibrationAdjustment = calibrationAdjustment

        control_type = ControlMode.Current
        self.control_mode: ControlMode = control_type
        self.raw_motor.setControlMode(control_type)
        self.raw_motor.setInputMode(
            control_type
        )  # input mode of current allows for all control types

        self.current_repeats = 0

    # ... rest of the class remains the same ...


    def getIP(self):
        return self.raw_motor.ip

    def enable(self):
        self.raw_motor.enable()

    def disable(self):
        self.raw_motor.setCurrent(0)
        self.valid = False
        self.raw_motor.disable()

    def getNonCalibratedCVP(self):
        with self.cvp_lock:
            if self.current_CVP is None:
                raise Exception("CVP not ready")
            return self.current_CVP
        
    def calibrateCVP(self, cvp: CVP):
        return CVP(cvp.current, cvp.velocity, cvp.position + self.calibrationAdjustment)    

    def getCVP(self) -> CVP:
        return self.calibrateCVP(self.getNonCalibratedCVP())

    def setNonCalibratedCVP(self, cvp: CVP):
        with self.cvp_lock:
            if self.current_CVP and self.current_CVP.current == cvp.current:
                self.current_repeats += 1

                if self.current_repeats > 10:
                    raise Exception("Error: Current not changing - check motor")
            else:
                self.current_repeats = 0
            
            self.current_CVP = cvp
            
        self.safetyConfiguration.check_within_limits(self.calibrateCVP(cvp))


    def requestReadyCheck(self):
        limit, margin = self.safetyConfiguration.get_velocity_limit_settings()
        positionP = self.configuration["positionP"]
        velocityP = self.configuration["velocityP"]
        velocityI = self.configuration["velocityI"]
        self.raw_motor.setPIDConfig(positionP, velocityP, velocityI, limit, margin)
        self.raw_motor.requestEncoderCheck()
        self.raw_motor.requestCVP()

    def cvpIsReady(self):
        with self.cvp_lock:
            return self.current_CVP is not None

    def confirmEncoderReady(self):
        with self.encoder_lock:
            self.encoder_ready = True

    def encoderIsReady(self):
        with self.encoder_lock:
            return self.encoder_ready

    def isReady(self):
        return self.encoderIsReady() and self.cvpIsReady()

    def setPosition(self, position: float):
        with self.config_lock:
            if not self.config_ready:
                return
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Position)

        cvp = self.getCVP()
        calibratedPosition = position - self.calibrationAdjustment
        override_value = self.safetyConfiguration.override_if_unsafe(
            cvp, ControlMode.Position, calibratedPosition
        )
        self.raw_motor.setPosition(override_value)

    def setVelocity(self, velocity: float):
        with self.config_lock:
            if not self.config_ready:
                return
        self.check_operatable()
        self.modeChangeIfNecessary(ControlMode.Velocity)

        cvp = self.getCVP()
        override_value = self.safetyConfiguration.override_if_unsafe(
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
        override_value = self.safetyConfiguration.override_if_unsafe(
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
