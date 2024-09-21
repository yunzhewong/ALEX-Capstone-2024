import os
import sys

package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from utils.Controllers import PIController
from utils.constants import (
    DAMPING_INTERCEPT,
    DAMPING_SLOPE,
    KINETIC_FRICTION,
    MOTOR_TORQUE_CONSTANT,
    STATIC_FRICTION,
    VELOCITY_ERROR_MULTIPLIER,
)
from utils.commands import CommandType, CommandObject
from utils.configreader import MotorConfiguration
from utils.constants import EPSILON


class MotorController:
    def __init__(self, config: MotorConfiguration):
        self.config = config
        self.positionController = PIController(config.position_p, 0)
        self.velocityController = PIController(
            config.vel_p * VELOCITY_ERROR_MULTIPLIER,
            config.vel_i * VELOCITY_ERROR_MULTIPLIER,
        )
        self.commandObject: CommandObject | None = None
        self.measuredPosition = 0
        self.measuredVelocity = 0
        self.motor = Motor()

    def setMeasurements(self, position, velocity):
        self.measuredPosition = position
        self.measuredVelocity = velocity

    def setCommand(self, commandObject: CommandObject):
        self.commandObject = commandObject

    def calculateOutputTorque(self, dt: float):
        if not self.commandObject:
            return 0, 0
        print("hello")

        reference = self.commandObject.value

        if self.commandObject.command == CommandType.Current:
            return self.motor.calculateOutput(reference, self.measuredVelocity)
        if self.commandObject.command == CommandType.Position:
            position_error = reference - self.measuredPosition
            velocity_control = self.positionController.compute_control(
                position_error, dt
            )

            velocity_error = velocity_control - self.measuredVelocity
            current_control = self.velocityController.compute_control(
                velocity_error, dt
            )
            return self.motor.calculateOutput(current_control, self.measuredVelocity)
        if self.commandObject.command == CommandType.Velocity:
            error = reference - self.measuredVelocity
            current_control = self.velocityController.compute_control(error, dt)
            return self.motor.calculateOutput(current_control, self.measuredVelocity)
        raise Exception("No Command")


class Motor:
    def __init__(self):
        pass

    def calculateOutput(
        self, inputCurrent: float, velocity: float
    ) -> tuple[float, float]:
        motorTorque = inputCurrent * MOTOR_TORQUE_CONSTANT
        dampingTorque = velocity * self.calculateDampingCoefficient(inputCurrent)
        inputTorque = motorTorque - dampingTorque
        return inputCurrent, self.calculateNetTorque(inputTorque, velocity)

    def calculateDampingCoefficient(self, inputCurrent: float):
        return DAMPING_SLOPE * abs(inputCurrent) + DAMPING_INTERCEPT

    def calculateNetTorque(self, inputTorque: float, velocity: float) -> float:
        # static friction is implemented via ROS URDF dynamics tag, where the friction is set as static friction
        # to recalibrate back to original dynamics, just give a boost to the torque when the link is moving

        if abs(velocity) < EPSILON:  # no motion
            return inputTorque
        return inputTorque - sign(inputTorque) * (STATIC_FRICTION - KINETIC_FRICTION)


def sign(val):
    if val >= 0:
        return 1
    return -1
