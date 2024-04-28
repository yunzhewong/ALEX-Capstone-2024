import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from alex_interfaces.srv import Command

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from commandTypes import CommandType
from classes.CommandObject import CommandObject

PROPELLOR_CONTROLLER = "propellor_controller"
FREQUENCY = 100
TIMER_PERIOD = 1 / FREQUENCY

MOTOR_TORQUE_CONSTANT = 0.1 # K_t
MOTOR_VOLTAGE_CONSTANT = 2 # K_c
MOTOR_RESISTANCE = 5 
MOTOR_VOLTAGE = 48
MOTOR_INDUCTANCE = 5 

GAIN_CONSTANT = 3


class PIDController():
    def __init__(self, K_p, K_i, K_d):
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d

        self.target = None

        self.previousTime = 0
        self.previousValue = 0

        self.e = 0
        self.e_int = 0
        self.e_dot = 0

    def clear(self):
        self.target = None
        self.e = 0
        self.e_int = 0
        self.e_dot = 0

         
    def setTarget(self, target):
        self.target = target
        self.e = 0
        self.e_int = 0
        self.e_dot = 0

    def updateLatest(self, time, value):
        if self.target is None:
            return
        self.e = value - self.target 
        self.e_dot = (value - self.previousValue) / (FREQUENCY * (time - self.previousTime))
        self.e_int += self.e * (1 / FREQUENCY)

        self.previousTime = time
        self.previousValue = value

    def getControlValue(self):
        return self.K_p * self.e + self.K_d * self.e_dot + self.K_i * self.e_int
        
class MotorController(Node):
    def __init__(self):
        self.positionPID = PIDController(5, 1, 3000)
        self.velocityPID = PIDController(1, 0, 10000)
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



class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        
        self.publisher = self.create_publisher(
            Float64MultiArray, f"/{PROPELLOR_CONTROLLER}/commands", 10
        )
        self.timer = self.create_timer(TIMER_PERIOD, self.publish_message)
        self.get_logger().info("Publisher Initialised")

        self.reader = self.create_subscription(
            JointState, "/joint_states", self.read_encoder, 10
        )
        self.get_logger().info("Reader Initialised")

        self.command_receiver = self.create_service(Command, 'command_receiver', self.respond_to_command)
        self.get_logger().info("Command Receiver Initialised")

        self.motorController = MotorController()
        self.index = 0

    def publish_message(self):
        torque = -1 * self.motorController.calculateTorque()

        msg = Float64MultiArray()
        msg.data = [float(torque)]
        msg.layout.data_offset = 0
        self.publisher.publish(msg)

    def read_encoder(self, msg: JointState):
        new_position = msg.position[0]
        self.motorController.updateState(self.index, new_position)
        self.index += 1

    def respond_to_command(self, request: Command.Request, response: Command.Response):
        self.motorController.updateCommand(CommandObject(request.command, request.value))

        self.get_logger().info(self.motorController.commandObject.toString())
        response.received = True
        return response

def main(args=None):
    rclpy.init(args=args)

    torque_publisher = MotorControllerNode()

    rclpy.spin(torque_publisher)

    torque_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
