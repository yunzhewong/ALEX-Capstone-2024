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
from constants import SEND_PERIOD
from qos import BestEffortQoS
from MotorController import MotorController
from configreader import read_config


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

        self.reader = self.create_subscription(
            JointState, "/joint_states", self.read_encoder, BestEffortQoS
        )
        self.get_logger().info("Reader Initialised")

        self.controllers: List[MotorController] = []
        counts, configs = read_config()
        for i in range(counts):
            self.controllers.append(MotorController(configs[i]))
        self.torque_publisher = Float64MultiArrayPublisher(self.create_publisher(
                Float64MultiArray, "/motor_controller/commands", BestEffortQoS
            ))
        self.get_logger().info("Torque Publisher Initialised")

        self.current_publisher = Float64MultiArrayPublisher(self.create_publisher(Float64MultiArray, f"/currents", BestEffortQoS))
        self.timer = self.create_timer(SEND_PERIOD, self.sendCommands)
        self.get_logger().info("Current Publisher Initialised")

        self.command_receiver = self.create_subscription(Command, '/commands', self.receive_command, BestEffortQoS)
        self.get_logger().info("Command Receiver Initialised")
        self.index = 0


    def sendCommands(self):
        currents = []
        torques = []
        for controller in self.controllers:
            torque = controller.calculateMotorTorque()
            
            currents.append(controller.calculateCurrent(torque))

            torque += controller.calculateFrictionAdjustment()
            torques.append(float(torque))
        self.current_publisher.publish(currents)
        self.torque_publisher.publish(torques)

    def read_encoder(self, msg: JointState):
        for i, controller in enumerate(self.controllers):
            new_position = msg.position[i]
            new_velocity = msg.velocity[i]
            controller.updateState(self.index, new_position, new_velocity)
        self.index += 1

    def receive_command(self, msg: Command):
        for i, controller in enumerate(self.controllers):
            ip = msg.ips[i]
            value = msg.values[i]
            command = msg.types[i]
            if controller.getIP() != ip:
                raise Exception("Invalid ip")
            controller.updateCommand(CommandObject(command, value))

def main(args=None):
    rclpy.init(args=args)
    torque_publisher = MotorControllerNode()
    rclpy.spin(torque_publisher)
    torque_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
