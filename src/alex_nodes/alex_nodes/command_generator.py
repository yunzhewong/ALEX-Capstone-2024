from enum import Enum
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

class State(Enum):
    Collecting = 1,
    Paused = 2,
    Resetting = 3
    

class CommandGenerator(Node):
    def __init__(self):
        super().__init__("command_generator")

        self.publisher = self.create_publisher(Command, "/commands", 10)
        self.publish_timer = self.create_timer(TIMER_PERIOD, self.send_command)

        self.subscriber = self.create_subscription(JointState, "/joint_states", self.read_time, 10)
        self.time = -1

        self.types = [CommandType.Current.value, CommandType.Current.value]
        self.values = [0.0, 0.0]

        self.positions = [0.0, 0.0]
        self.velocities = [0.0, 0.0]

        self.initialised = False
        self.start_time = -1

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
        if self.time < 0:
            return

        if not self.initialised:
            self.start_time = t
            self.initialised = True

        DURATION = 50
        WAVE_MAGNITUDE = 3
        INITIAL_FREQUENCY = 0
        FINAL_FREQUENCY = 25
        CHIRP_RATE = (FINAL_FREQUENCY - INITIAL_FREQUENCY) / DURATION

        runningTime = t - self.start_time

        if runningTime > DURATION:
            self.types = [CommandType.Current.value, CommandType.Position.value]
            self.values = [0.0, 0.0]
            return

        frequency = CHIRP_RATE * runningTime + INITIAL_FREQUENCY
        current = WAVE_MAGNITUDE * math.sin(2 * math.pi * frequency * runningTime)

        self.types = [CommandType.Current.value, CommandType.Current.value]
        self.values = [0.0, current]
        
        
  

def main(args=None):
    rclpy.init(args=args)
    command_generator = CommandGenerator()
    rclpy.spin(command_generator)
    command_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
