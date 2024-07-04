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

        self.start_reset = -1
        self.end_reset = -1
        self.pause = -1
        self.start_current = 0.5
        self.increment = 0.1
        self.index = 0
        self.reading = False
        self.start_time = -1
        self.reset_angle = -1

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
        MAX_ANGLE = math.pi / 2
        MAX_TIME = 10
        STARTING_ANGLE = -math.pi / 2
        RESET_TIME = 5
        PAUSE_TIME = 1
        
        if len(self.positions) < 2:
            return
        
        should_stop = self.positions[1] >= MAX_ANGLE
    
        if t > self.pause and should_stop:
            self.pause = t + PAUSE_TIME
            self.start_reset = self.pause
            self.end_reset = self.start_reset + RESET_TIME
            self.reset_angle = self.positions[1]

        if t >= self.start_reset and t <= self.end_reset:
            expected_pos = self.reset_angle - ((self.reset_angle - STARTING_ANGLE) / RESET_TIME) * (t - self.start_reset)

            self.types = [CommandType.Current.value, CommandType.Position.value]
            self.values = [0.0, float(expected_pos)]
            self.pause = self.end_reset + PAUSE_TIME
            if self.reading:
                self.index += 1
                self.reading = False
        elif t <= self.pause:
            self.types = [CommandType.Current.value, CommandType.Current.value] 
            self.values = [0.0, 0.0]
        else:
            command = self.start_current + self.increment * self.index
            if not self.reading:
                print(command)
                self.start_time = t
                self.reading = True

            self.types = [CommandType.Current.value, CommandType.Current.value] 
            self.values = [0.0, command]

def main(args=None):
    rclpy.init(args=args)
    command_generator = CommandGenerator()
    rclpy.spin(command_generator)
    command_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
