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


PROPELLOR_CONTROLLER = "propellor_controller"
FREQUENCY = 100

MOTOR_CONSTANT = 0.1
GAIN_CONSTANT = 3

class CommandInfo():
    def __init__(self, command, value):
        self.command = command
        self.value = value

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        
        self.publisher = self.create_publisher(
            Float64MultiArray, f"/{PROPELLOR_CONTROLLER}/commands", 10
        )
        timer_period = 1 / FREQUENCY  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.get_logger().info("Publisher Initialised")

        self.reader = self.create_subscription(
            JointState, "/joint_states", self.read_encoder, 10
        )
        self.position = None
        self.get_logger().info("Reader Initialised")

        self.command_receiver = self.create_service(Command, 'command_receiver', self.respond_to_command)
        self.command_info = None
        self.get_logger().info("Command Receiver Initialised")

    def calculate_torque(self, command_info: CommandInfo | None, position: int | None):
        if command_info is None or position is None:
            return 0
        
        if command_info.command == CommandType.CURRENT:
            return MOTOR_CONSTANT * command_info.value
        if command_info.command == CommandType.POSITION:
            return -3 * (position - command_info.value)
        return 0

    def publish_message(self):
        torque = self.calculate_torque(self.command_info, self.position)

        msg = Float64MultiArray()
        msg.data = [float(torque)]
        msg.layout.data_offset = 0
        self.publisher.publish(msg)

    def read_encoder(self, msg: JointState):
        self.position = msg.position[0]

    def respond_to_command(self, request: Command.Request, response: Command.Response):
        response.received = True

        if request.command == CommandType.POSITION:
            self.get_logger().info(f"Position Control: {request.value}")
        elif request.command == CommandType.VELOCITY:
            self.get_logger().info(f"Velocity Control: {request.value}")
        elif request.command == CommandType.CURRENT:
            self.get_logger().info(f"Current Control: {request.value}")    
        self.command_info = CommandInfo(request.command, request.value)   
        return response

def main(args=None):
    rclpy.init(args=args)

    torque_publisher = MotorController()

    rclpy.spin(torque_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    torque_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
