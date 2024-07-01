import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from motor_pubsub_utils.constants import TIMER_PERIOD

class CommandGenerator(Node):
    def __init__(self):
        super().__init__("command_generator")

        self.publisher = self.create_publisher(String, f"/commands", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.send_command)
        self.count = 0

    def send_command(self):
        
        msg = String()
        msg.data = f"Hello World {self.count}"
        self.publisher.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    command_generator = CommandGenerator()

    rclpy.spin(command_generator)

    command_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
