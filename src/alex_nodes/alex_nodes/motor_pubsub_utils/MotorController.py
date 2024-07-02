from alex_nodes.motor_pubsub_utils.PID import PIDController
from alex_nodes.motor_pubsub_utils.Commands import CommandObject
from alex_nodes.commandTypes import CommandType
from alex_nodes.motor_pubsub_utils.constants import MOTOR_TORQUE_CONSTANT, POSITION_GAIN, VEL_GAIN, VEL_INTEGRATOR_GAIN

class MotorController():
    def __init__(self, ip):
        self.ip = ip
        self.positionPID = PIDController(POSITION_GAIN, 0, 0)
        self.velocityPID = PIDController(0.5 * VEL_GAIN / VEL_GAIN, VEL_INTEGRATOR_GAIN, 0)
        self.commandObject = None

    def updateCommand(self, commandObject: CommandObject):
        self.positionPID.clear()
        self.velocityPID.clear()
        self.commandObject = commandObject
        if (commandObject.command == CommandType.Current):
            return
        if (commandObject.command == CommandType.Position):
            self.positionPID.setTarget(commandObject.value)
            return
        if (commandObject.command == CommandType.Velocity):
            self.velocityPID.setTarget(commandObject.value)
            return
        raise Exception("Invalid Command")

    def updateState(self, time, position, velocity):
        self.positionPID.updateLatest(time, position)
        self.velocityPID.updateLatest(time, velocity)

    def calculateTorque(self):
        if not self.commandObject:
            return 0
        
        if (self.commandObject.command == CommandType.Current):
            return MOTOR_TORQUE_CONSTANT * self.commandObject.value
        if (self.commandObject.command == CommandType.Position):
            return self.positionPID.getControlValue()
        if (self.commandObject.command == CommandType.Velocity):
            return self.velocityPID.getControlValue()
        raise Exception("No Command")

