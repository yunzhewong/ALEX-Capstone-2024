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

from constants import SEND_PERIOD
from twinmotor import IPS
from commands import CommandType

class State(Enum):
    Collecting = 1,
    Paused = 2,
    Resetting = 3
    

class CommandGenerator(Node):
    def __init__(self):
        super().__init__("command_generator")

        self.publisher = self.create_publisher(Command, "/commands", 10)
        self.publish_timer = self.create_timer(SEND_PERIOD, self.send_command)

        self.subscriber = self.create_subscription(JointState, "/joint_states", self.read_time, 10)
        self.time = -1

        self.types = [CommandType.Current.value, CommandType.Current.value]
        self.values = [0.0, 0.0]

        self.positions = [0.0, 0.0]
        self.velocities = [0.0, 0.0]

        self.initialised = False

        self.state = State.Collecting
        self.post_pause_state = State.Resetting
        self.collecting_start = -1
        self.collect_index = 0
        self.pause_end_time = -1
        self.reset_start = -1
        self.reset_end = -1
        self.reset_angle = -1
        self.start_time = -1


    def send_command(self):
        self.commands(self.time)
        
        msg = Command()
        msg.ips = IPS
        msg.types = self.types
        msg.values = self.values
        self.publisher.publish(msg)

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
        START_CURRENT = 0.1
        INCREMENT = 0.1
        
        if len(self.positions) < 2:
            return
        
        self.types = [CommandType.Current.value, CommandType.Current.value] 
        self.values = [0.0, 1.0]

        # if self.state == State.Collecting:
        #     command = START_CURRENT + INCREMENT * self.collect_index
        #     if not self.initialised:
        #         self.collecting_start = t
        #         self.initialised = True
        #         print(command)

        #     exceeded_max_angle = self.positions[1] >= MAX_ANGLE
        #     exceeded_max_time = (t - self.collecting_start) >= MAX_TIME

        #     should_stop = exceeded_max_angle or exceeded_max_time

        #     if should_stop:
        #         self.state = State.Paused
        #         self.post_pause_state = State.Resetting
        #         self.collect_index += 1
        #         self.initialised = False

        #     self.types = [CommandType.Current.value, CommandType.Current.value] 
        #     self.values = [0.0, command]
        # elif self.state == State.Paused:
        #     if not self.initialised:
        #         self.pause_end_time = t + PAUSE_TIME
        #         self.initialised = True

        #     if t > self.pause_end_time:
        #         self.state = self.post_pause_state
        #         self.initialised = False

        #     self.types = [CommandType.Current.value, CommandType.Current.value] 
        #     self.values = [0.0, 0.0]
        # elif self.state == State.Resetting:
        #     if not self.initialised:
        #         self.reset_start = t
        #         self.reset_end = t + RESET_TIME
        #         self.reset_angle = self.positions[1]
        #         self.initialised = True

        #     expected_pos = self.reset_angle - ((self.reset_angle - STARTING_ANGLE) / RESET_TIME) * (t - self.reset_start)

        #     if t > self.reset_end:
        #         self.post_pause_state = State.Collecting
        #         self.state = State.Paused
        #         self.initialised = False
            
        #     self.types = [CommandType.Current.value, CommandType.Position.value] 
        #     self.values = [0.0, float(expected_pos)]

        
  

def main(args=None):
    rclpy.init(args=args)
    command_generator = CommandGenerator()
    rclpy.spin(command_generator)
    command_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
