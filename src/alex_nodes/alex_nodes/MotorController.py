import os
import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from PID import PIDController
from commands import CommandType, CommandObject
from constants import EPSILON, FRICTION_ADJUSTMENT, MOTOR_TORQUE_CONSTANT, POSITION_GAIN, VEL_GAIN, VEL_INTEGRATOR_GAIN

class MotorController():
    def __init__(self, ip):
        self.ip = ip
        self.positionPID = PIDController(POSITION_GAIN / POSITION_GAIN * 0.1, 0, 0)
        self.velocityPID = PIDController(0.5 * VEL_GAIN / VEL_GAIN, VEL_INTEGRATOR_GAIN, 0)
        self.commandObject = None
        self.velocity = 0

    def updateCommand(self, commandObject: CommandObject):
        if self.commandObject:
            matchingCommand = commandObject.command == self.commandObject.command
            matchingValue = commandObject.value == self.commandObject.value
            if matchingCommand and matchingValue:
                return  

        self.commandObject = commandObject
        if (commandObject.command == CommandType.Current):
            return
        if (commandObject.command == CommandType.Position):
            self.positionPID.setReference(commandObject.value)
            return
        if (commandObject.command == CommandType.Velocity):
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
        
        if (self.commandObject.command == CommandType.Current):
            return MOTOR_TORQUE_CONSTANT * self.commandObject.value
        if (self.commandObject.command == CommandType.Position):
            return self.positionPID.getControlValue()
        if (self.commandObject.command == CommandType.Velocity):
            return self.velocityPID.getControlValue()
        raise Exception("No Command")

    def calculateFrictionAdjustment(self):
        if abs(self.velocity) < EPSILON:
            return 0
        return 0

        if self.velocity > 0:
            return FRICTION_ADJUSTMENT 
        return -FRICTION_ADJUSTMENT