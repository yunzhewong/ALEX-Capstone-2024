from enum import Enum
import math
import os
from typing import List
import rclpy
from rclpy.node import Node
from alex_interfaces.msg import Command
from sensor_msgs.msg import JointState


import sys

package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from utils.constants import MOTOR_NETWORKING_PERIOD
from utils.commands import CommandType
from utils.qos import BestEffortQoS
from utils.configreader import read_config
import utils.command_list as command_list
import utils.ros as ros


class CommandGenerator(Node):
    def __init__(self):
        super().__init__("command_generator")

        self.motor_count, self.motor_configs = read_config()
        self.ips = [config.ip for config in self.motor_configs]

        self.publisher = self.create_publisher(Command, "/commands", BestEffortQoS)
        self.publish_timer = self.create_timer(
            MOTOR_NETWORKING_PERIOD, self.send_command
        )

        self.subscriber = self.create_subscription(
            JointState, "/joint_states", self.read_time, BestEffortQoS
        )

        self.readings = JointReadings()
        self.controllers = [CascadeController() for _ in range(6)]
        self.init_time = -1

        self.trajectory = command_list.Exo24Trajectory()

    def send_command(self):
        if self.init_time == -1:
            return

        runningTime = self.readings.get_time() - self.init_time
        positions, velocities = self.trajectory.get_state(runningTime)

        output_velocities = []
        for i in range(6):
            measured_pos, _ = self.readings.get_reading(i)
            dt = self.readings.get_iteration_time()

            velocity_command = self.controllers[i].calculate_velocity(
                measured_pos, positions[i], velocities[i], dt
            )
            output_velocities.append(velocity_command)
        print(output_velocities)

        msg = Command()
        msg.ips = self.ips
        msg.types = [CommandType.Velocity.value for _ in range(6)]
        msg.values = output_velocities
        self.publisher.publish(msg)

    def read_time(self, msg: JointState):
        self.readings.set_readings(msg)
        if self.init_time == -1:
            self.init_time = self.readings.get_time()


class JointReadings:
    def __init__(self):
        self.last_time = 0
        self.time = 0
        self.velocities = []
        self.positions = []

    def get_time(self):
        return self.time

    def get_iteration_time(self):
        return self.time - self.last_time

    def get_reading(self, index: int):
        return self.positions[index], self.velocities[index]

    def set_readings(self, msg: JointState):
        self.last_time = self.time
        self.time = ros.decode_time(msg)
        self.positions = msg.position
        self.velocities = msg.velocity


CONTROLLER_P_GAIN = 10
CONTROLLER_I_GAIN = 0.1


class PIController:
    def __init__(self, Kp: float, Ki: float):
        self.Kp = Kp
        self.Ki = Ki
        self.integral_error = 0

    def compute_control(self, error: float, dt: float):
        self.integral_error += error * dt
        return self.Kp * error + self.Ki * self.integral_error


class CascadeController:
    def __init__(self):
        self.outer_controller = PIController(CONTROLLER_P_GAIN, CONTROLLER_I_GAIN)

    def calculate_velocity(
        self,
        position: float,
        reference_position: float,
        reference_velocity: float,
        dt: float,
    ):
        position_error = reference_position - position
        position_correction = self.outer_controller.compute_control(position_error, dt)
        return reference_velocity + position_correction


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
