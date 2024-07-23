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
from commands import CommandType
from qos import BestEffortQoS
from configreader import read_config
import command_list


class CommandGenerator(Node):
    def __init__(self):
        super().__init__("command_generator")

        self.motor_count, self.motor_configs = read_config()
        self.ips = [config.ip for config in self.motor_configs]

        self.publisher = self.create_publisher(Command, "/commands", BestEffortQoS)
        self.publish_timer = self.create_timer(SEND_PERIOD, self.send_command)

        self.subscriber = self.create_subscription(
            JointState, "/joint_states", self.read_time, BestEffortQoS
        )
        self.time = -1
        self.types = []
        self.values = []
        self.positions = []
        self.velocities = []
        self.initialised = False
        self.init_time = -1

        self.dt = 0.02
        positions, velocities = command_list.exo_demo_trajectory(self.dt)
        self.trajectories = positions

    def send_command(self):
        if not self.initialised:
            return
        self.commands(self.time - self.init_time)

        msg = Command()
        msg.ips = self.ips
        msg.types = self.types
        msg.values = self.values
        self.publisher.publish(msg)

    def read_time(self, msg: JointState):
        timestamp = msg.header.stamp
        self.time = timestamp.sec + timestamp.nanosec / 1e9
        self.positions = msg.position
        self.velocities = msg.velocity
        if not self.initialised:
            self.init_time = self.time
            self.initialised = True

    def commands(self, t):
        if len(self.positions) < self.motor_count:
            raise Exception("Too few motors")
        print(t)

        command_list.follow_demo_trajectory(self, t)


class DataLog:
    def open(self, name: str):
        self.f = open(name, "w")
        self.f.write("Time, Current, Velocity, Position\n")

    def write(self, t: float, c: float, v: float, p: float):
        self.f.write(f"{t}, {c}, {v}, {p}\n")

    def close(self):
        self.f.close()


def main(args=None):
    rclpy.init(args=args)
    command_generator = CommandGenerator()
    rclpy.spin(command_generator)
    command_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
