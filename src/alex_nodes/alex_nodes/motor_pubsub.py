import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from alex_interfaces.srv import Command

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

import commandTypes as CommandType
from classes.CommandObject import CommandObject

PROPELLOR_CONTROLLER = "propellor_controller"
FREQUENCY = 100
TIMER_PERIOD = 1 / FREQUENCY

MOTOR_CONSTANT = 0.1
GAIN_CONSTANT = 3


class MotorController(Node):
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
        self.position = None
        self.velocity = None
        self.get_logger().info("Reader Initialised")

        self.command_receiver = self.create_service(Command, 'command_receiver', self.respond_to_command)
        self.command_info = None
        self.get_logger().info("Command Receiver Initialised")

    def calculate_torque(self, command_info: CommandObject | None, position: int | None, velocity: int | None):
        if command_info is None:
            return 0
        
        if command_info.command == CommandType.CURRENT:
            return MOTOR_CONSTANT * command_info.value
        if command_info.command == CommandType.VELOCITY:
            return -1 * (velocity - command_info.value)
        if command_info.command == CommandType.POSITION:
            if position is None:
                return 0
            return -3 * (position - command_info.value)
        return 0

    def publish_message(self):
        torque = self.calculate_torque(self.command_info, self.position, self.velocity)

        msg = Float64MultiArray()
        msg.data = [float(torque)]
        msg.layout.data_offset = 0
        self.publisher.publish(msg)

    def read_encoder(self, msg: JointState):
        new_position = msg.position[0]

        if self.position is not None:
            self.velocity = (new_position - self.position) / TIMER_PERIOD
        self.position = new_position

    def respond_to_command(self, request: Command.Request, response: Command.Response):
        response.received = True
        self.command_info = CommandObject(request.command, request.value)   
        self.get_logger().info(self.command_info.toString())
        return response

def main(args=None):
    rclpy.init(args=args)

    torque_publisher = MotorController()

    rclpy.spin(torque_publisher)

    torque_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
