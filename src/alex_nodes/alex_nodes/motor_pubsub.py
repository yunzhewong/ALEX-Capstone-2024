import os
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from alex_interfaces.msg import Command

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from commands import CommandObject
from constants import MOTOR_TORQUE_CONSTANT, SIMULATION_PERIOD
from MotorController import MotorController


EXPECTED_IPS = ["10.10.10.17", "10.10.10.16"]
IP_MAP = {
    "10.10.10.17": "bottom_motor",
    "10.10.10.16": "top_motor"
}

class Float64MultiArrayPublisher():
    def __init__(self, ros_publisher):
        self.publisher = ros_publisher

    def publish(self, values: List[float]):
        msg = Float64MultiArray()
        msg.data = values
        msg.layout.data_offset = 0
        self.publisher.publish(msg)

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller")

        self.motor_pairs: List[tuple[Float64MultiArrayPublisher, MotorController]] = []

        self.reader = self.create_subscription(
            JointState, "/joint_states", self.read_encoder, 10
        )
        self.get_logger().info("Reader Initialised")
        
        for ip in EXPECTED_IPS:
            publisher = Float64MultiArrayPublisher(self.create_publisher(
                Float64MultiArray, f"/{IP_MAP[ip]}_controller/commands", 10
            ))
            controller = MotorController(ip)
            self.motor_pairs.append((publisher, controller))

        self.current_publisher = Float64MultiArrayPublisher(self.create_publisher(Float64MultiArray, f"/currents", 10))
        self.timer = self.create_timer(SIMULATION_PERIOD, self.sendCommands)
        self.get_logger().info("Publisher Initialised")

        self.command_receiver = self.create_subscription(Command, '/commands', self.receive_command, 10)
        self.get_logger().info("Command Receiver Initialised")
        self.index = 0


    def sendCommands(self):
        currents = []
        for (publisher, controller) in self.motor_pairs:
            torque = controller.calculateMotorTorque()
            current = torque / MOTOR_TORQUE_CONSTANT
            currents.append(current)
            # torque += controller.calculateFrictionAdjustment()
            publisher.publish([float(torque)])
        self.current_publisher.publish(currents)

    def read_encoder(self, msg: JointState):
        for i, (_, controller) in enumerate(self.motor_pairs):
            new_position = msg.position[i]
            new_velocity = msg.velocity[i]
            controller.updateState(self.index, new_position, new_velocity)
        self.index += 1

    def receive_command(self, msg: Command):
        for i, ip in enumerate(msg.ips):
            for (_, controller) in self.motor_pairs:
                if controller.ip == ip:
                    command = msg.types[i] 
                    value = msg.values[i]
                    controller.updateCommand(CommandObject(command, value))

def main(args=None):
    rclpy.init(args=args)
    torque_publisher = MotorControllerNode()
    rclpy.spin(torque_publisher)
    torque_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
