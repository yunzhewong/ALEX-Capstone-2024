from alex_nodes.motor_pubsub_utils.PID import PIDController
from alex_nodes.classes.Commands import CommandObject
from alex_nodes.commandTypes import CommandType
from alex_nodes.motor_pubsub_utils.constants import MOTOR_TORQUE_CONSTANT, POSITION_GAIN, VEL_GAIN, VEL_INTEGRATOR_GAIN
from std_msgs.msg import Float64MultiArray


class MotorController():
    def __init__(self, ip, publisher):
        self.ip = ip
        self.publisher = publisher
        self.positionPID = PIDController(POSITION_GAIN, 0, 0)
        self.velocityPID = PIDController(VEL_GAIN, VEL_INTEGRATOR_GAIN, 0)
        self.commandObject = None

    def updateCommand(self, commandObject: CommandObject):
        self.positionPID.clear()
        self.velocityPID.clear()
        self.commandObject = commandObject
        if (commandObject.command == CommandType.Current):
            return
        
        if (commandObject.command == CommandType.Position):
            self.positionPID.setTarget(commandObject.value)
        else:
            self.velocityPID.setTarget(commandObject.value)

    def updateState(self, time, position):
        self.positionPID.updateLatest(time, position)
        self.velocityPID.updateLatest(time, self.positionPID.e_dot)

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
    
    def sendActuationCommand(self):
        torque = -1 * self.calculateTorque()
        msg = Float64MultiArray()
        msg.data = [float(torque)]
        msg.layout.data_offset = 0
        self.publisher.publish(msg)

