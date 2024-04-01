import os

from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from alex_interfaces.srv import Command

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)


PROPELLOR_CONTROLLER = "propellor_controller"


class StateReader(Node):
    def __init__(self):
        super().__init__("state_reader")
        
        _, axs = plt.subplots(2)
        self.axs = axs

        self.torque_sub = self.create_subscription(
            Float64MultiArray, f"/{PROPELLOR_CONTROLLER}/commands", self.read_torque, 10
        )
        self.position_sub = self.create_subscription(
            JointState, "/joint_states", self.read_position, 10
        )
        self.times = []
        self.torques = []
        self.positions = []
        self.velocities = []

        

    def read_position(self, msg: JointState):
        
        timestamp = msg.header.stamp
        val = timestamp.sec + timestamp.nanosec / 1e9
        self.times.append(val)
        self.positions.append(msg.position[0])
        self.velocities.append(msg.velocity[0])
        self.update()
        
    def read_torque(self, msg: Float64MultiArray):
        self.torques.append(msg.data[0])

    def update(self):
        self.axs[0].cla()
        self.axs[1].cla()

        self.axs[0].plot(self.times, self.positions)
        self.axs[0].set_title("Positions (rad)")
        self.axs[1].plot(self.times, self.velocities)
        self.axs[1].set_title("Velocities (rad/s)")
        plt.tight_layout()
        plt.pause(0.01)
def main(args=None):
    rclpy.init(args=args)

    state_reader = StateReader()

    rclpy.spin(state_reader)

    state_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
