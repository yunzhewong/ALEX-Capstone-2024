import os
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from alex_interfaces.srv import Command

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from alex_nodes.classes.Commands import CommandObject
from motor_pubsub_utils.constants import TIMER_PERIOD
from motor_pubsub_utils.MotorController import MotorController

PROPELLOR_CONTROLLER = "propellor_controller"

EXPECTED_IPS = ["10.10.10.16", "10.10.10.17"]
IP_MAP = {
    "10.10.10.16": "motor1",
    "10.10.10.17": "motor2"
}

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller")

        self.motorControllers: List[MotorController] = []

        for ip in EXPECTED_IPS:
            publisher = self.create_publisher(
                Float64MultiArray, f"/{IP_MAP[ip]}_controller/commands", 10
            )
            motorController = MotorController(ip, publisher)
            self.motorControllers.append(motorController)
        self.timer = self.create_timer(TIMER_PERIOD, self.sendCommands)
        self.get_logger().info("Publisher Initialised")

        self.reader = self.create_subscription(
            JointState, "/joint_states", self.read_encoder, 10
        )
        self.get_logger().info("Reader Initialised")

        self.command_receiver = self.create_service(Command, 'command_receiver', self.respond_to_command)
        self.get_logger().info("Command Receiver Initialised")

        self.index = 0

    def sendCommands(self):
        for controller in self.motorControllers:
            controller.sendActuationCommand()

    def read_encoder(self, msg: JointState):
        for i, controller in enumerate(self.motorControllers):
            new_position = msg.position[i]
            new_velocity = msg.velocity[i]
            controller.updateState(self.index, new_position, new_velocity)
        self.index += 1

    def respond_to_command(self, request: Command.Request, response: Command.Response):
        for controller in self.motorControllers:
            if controller.ip == request.ip:
                controller.updateCommand(CommandObject(request.command, request.value))
                self.get_logger().info(f"{controller.ip} -> {controller.commandObject.toString()}")
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
