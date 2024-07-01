import math
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from alex_interfaces.msg import Command
import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from motor_pubsub_utils.constants import TIMER_PERIOD
from twinmotor import IPS
from commandTypes import CommandType

class CommandGenerator(Node):
    def __init__(self):
        super().__init__("command_generator")

        self.publisher = self.create_publisher(Command, f"/commands", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.send_command)
        self.time = 0

        self.types = [CommandType.Current.value, CommandType.Current.value]
        self.values = [0, 0]

    def send_command(self):
        self.commands(self.time)
        
        msg = Command()
        msg.ips = IPS
        msg.types = self.types
        msg.values = self.values
        self.publisher.publish(msg)
        self.time += TIMER_PERIOD

    def commands(self, t):
        val = 5 * math.sin(2 * math.pi * 0.1 * t)
        self.types = [CommandType.Current.value, CommandType.Current.value] 
        self.values = [val, -val]

def main(args=None):
    rclpy.init(args=args)
    command_generator = CommandGenerator()
    rclpy.spin(command_generator)
    command_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()