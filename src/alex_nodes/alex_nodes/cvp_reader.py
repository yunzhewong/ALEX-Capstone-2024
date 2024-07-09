import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

class CVPReader(Node):
    def __init__(self):
        super().__init__("cvp_reader")
        self.joints_sub = self.create_subscription(JointState, "/joint_states", self.read_joints, 10)
        self.current_sub = self.create_subscription(Float64MultiArray, "/currents", self.read_currents, 10)

        self.time = 0
        self.currents = []
        self.velocities = []
        self.positions = []

        self.f = open("data.csv", "w")
        self.written = False

    def read_joints(self, msg: JointState):
        timestamp = msg.header.stamp
        self.time = timestamp.sec + timestamp.nanosec / 1e9
        names = msg.name
        self.positions = msg.position
        self.velocities = msg.velocity

        if len(self.currents) == 0 or len(self.currents) != len(self.positions):
            return
        
        if not self.written:
            self.f.write("Time")
            for name in names:
                self.f.write(f", {name} Current, {name} Velocity, {name} Position")
            self.f.write("\n")
            self.written = True

        self.f.write(f"{self.time}")
        for i in range(len(names)):
            self.f.write(f", {self.currents[i]}, {self.velocities[i]}, {self.positions[i]}")
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
