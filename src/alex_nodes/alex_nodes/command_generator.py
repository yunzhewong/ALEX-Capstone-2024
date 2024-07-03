import math
import os
import rclpy
from rclpy.node import Node
from alex_interfaces.msg import Command
from sensor_msgs.msg import JointState

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from motor_pubsub_utils.constants import TIMER_PERIOD
from twinmotor import IPS
from commandTypes import CommandType

class CommandGenerator(Node):
    def __init__(self):
        super().__init__("command_generator")

        self.publisher = self.create_publisher(Command, "/commands", 10)
        self.publish_timer = self.create_timer(TIMER_PERIOD, self.send_command)

        self.subscriber = self.create_subscription(JointState, "/joint_states", self.read_time, 10)
        self.time = 0

        self.types = [CommandType.Current.value, CommandType.Current.value]
        self.values = [0, 0]

        self.positions = [0, 0]
        self.velocities = [0, 0]

    def send_command(self):
        self.commands(self.time)
        
        msg = Command()
        msg.ips = IPS
        msg.types = self.types
        msg.values = self.values
        self.publisher.publish(msg)
        self.time += TIMER_PERIOD

    def read_time(self, msg: JointState):
        timestamp = msg.header.stamp
        self.time = timestamp.sec + timestamp.nanosec / 1e9
        self.positions = msg.position
        self.velocities = msg.velocity

    def commands(self, t):
        self.types = [CommandType.Current.value, CommandType.Current.value] 
        self.values = [0.0, 1.0]

def main(args=None):
    rclpy.init(args=args)
    command_generator = CommandGenerator()
    rclpy.spin(command_generator)
    command_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
