from alex_nodes.motor_pubsub_utils.PID import PIDController
from alex_nodes.commands import CommandType, CommandObject
from alex_nodes.motor_pubsub_utils.constants import MOTOR_TORQUE_CONSTANT, POSITION_GAIN, VEL_GAIN, VEL_INTEGRATOR_GAIN

class MotorController():
    def __init__(self, ip):
        self.ip = ip
        self.positionPID = PIDController(POSITION_GAIN, 0, 0)
        self.velocityPID = PIDController(0.5 * VEL_GAIN / VEL_GAIN, VEL_INTEGRATOR_GAIN, 0)
        self.commandObject = None

    def updateCommand(self, commandObject: CommandObject):
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

    def calculateTorque(self):
        if not self.commandObject:
            return 0
        
        if (self.commandObject.command == CommandType.Current):
            return MOTOR_TORQUE_CONSTANT * self.commandObject.value
        if (self.commandObject.command == CommandType.Position):
            return -1 * self.positionPID.getControlValue()
        if (self.commandObject.command == CommandType.Velocity):
            return -1 * self.velocityPID.getControlValue()
        raise Exception("No Command")

