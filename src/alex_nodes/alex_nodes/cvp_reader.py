import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import sys

package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from utils.qos import BestEffortQoS
from utils.configreader import read_config
from utils.JointReadings import JointReadings


class CVPReader(Node):
    def __init__(self):
        super().__init__("cvp_reader")

        _, self.motor_configs = read_config()
        self.names = [config.name for config in self.motor_configs]
        self.joints_sub = self.create_subscription(
            JointState, "/joint_states", self.read_joints, BestEffortQoS
        )
        self.current_sub = self.create_subscription(
            Float64MultiArray, "/currents", self.read_currents, BestEffortQoS
        )

        self.readings = JointReadings()

        self.f = open("data.csv", "w")
        self.f.write("Time")
        for name in self.names:
            self.f.write(f", {name} Current, {name} Velocity, {name} Position")
        self.f.write("\n")
        self.written = True

    def read_joints(self, msg: JointState):
        self.readings.set_readings(msg)

        self.f.write(f"{self.readings.get_time()}")
        for i in range(len(self.names)):
            position, velocity = self.readings.get_reading(i)
            self.f.write(f", {self.currents[i]}, {velocity}, {position}")
        self.f.write("\n")

    def read_currents(self, msg: Float64MultiArray):
        self.currents = msg.data

    def close(self):
        self.f.close()


def main(args=None):
    rclpy.init(args=args)
    cvp_reader = CVPReader()
    rclpy.spin(cvp_reader)
    cvp_reader.close()
    cvp_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
