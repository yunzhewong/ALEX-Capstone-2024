import os
import sys

package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from utils.PID import PIDController
from utils.commands import CommandType, CommandObject
from utils.configreader import MotorConfiguration
from utils.constants import EPSILON


class MotorController:
    def __init__(self, config: MotorConfiguration):
        self.config = config
        self.positionPID = PIDController(config.position_p, 0, 0)
        self.velocityPID = PIDController(config.vel_p, config.vel_i, 0)
        self.commandObject = None
        self.velocity = 0

    def getIP(self):
        return self.config.ip

    def updateCommand(self, commandObject: CommandObject):
        if self.commandObject:
            matchingCommand = commandObject.command == self.commandObject.command
            matchingValue = commandObject.value == self.commandObject.value
            if matchingCommand and matchingValue:
                return

        self.commandObject = commandObject
        if commandObject.command == CommandType.Current:
            return
        if commandObject.command == CommandType.Position:
            self.positionPID.setReference(commandObject.value)
            return
        if commandObject.command == CommandType.Velocity:
            self.velocityPID.setReference(commandObject.value)
            return
        raise Exception("Invalid Command")

    def updateState(self, time, position, velocity):
        self.positionPID.update(time, position)
        self.velocityPID.update(time, velocity)
        self.velocity = velocity

    def calculateMotorTorque(self):
        if not self.commandObject:
            return 0

        if self.commandObject.command == CommandType.Current:
            return self.config.motor_constant * self.commandObject.value
        if self.commandObject.command == CommandType.Position:
            return self.positionPID.getControlValue()
        if self.commandObject.command == CommandType.Velocity:
            return self.velocityPID.getControlValue()
        raise Exception("No Command")

    def calculateCurrent(self, torque):
        return torque / self.config.motor_constant

    def calculateFrictionAdjustment(self):
        if abs(self.velocity) < EPSILON:
            return 0

        if self.velocity > 0:
            return self.config.friction_adjustment
        return -self.config.friction_adjustment
