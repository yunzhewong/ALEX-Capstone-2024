import os

from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from alex_interfaces.srv import Command
import numpy as np
import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

class StateReader(Node):
    def __init__(self):
        super().__init__("encoder_reader")
        
        self.position_sub = self.create_subscription(
            JointState, "/joint_states", self.read_position, 10
        )
        self.times = []
        self.positions = []
        self.update()

    def read_position(self, msg: JointState):
        timestamp = msg.header.stamp
        val = timestamp.sec + timestamp.nanosec / 1e9
        self.times.append(val)
        self.positions.append(msg.position[0])
        self.update()

    def update(self):
        plt.cla()
        plt.plot(self.times, self.positions)
        plt.title("Positions (rad)")
        plt.pause(0.01)

    def export_data(self):
        npTimes = np.array(self.times)
        npPositions = np.array(self.positions)
        return np.column_stack((npTimes, npPositions))

def main(args=None):
    rclpy.init(args=args)

    state_reader = StateReader()

    rclpy.spin(state_reader)
    
    data = state_reader.export_data()
    np.savetxt('data.csv', data, delimiter=',', header='times(s), positions(rads)', comments="")

    state_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
