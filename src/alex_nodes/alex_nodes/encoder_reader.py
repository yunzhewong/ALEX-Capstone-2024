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
        _, axs = plt.subplots(2, 1)
        self.axs = axs
        
        self.position_sub = self.create_subscription(
            JointState, "/joint_states", self.read_position, 10
        )
        self.times = []
        self.positions = []
        self.velocities = []
        self.update()

    def read_position(self, msg: JointState):
        timestamp = msg.header.stamp
        time = timestamp.sec + timestamp.nanosec / 1e9

        newPosition = msg.position[0]
        if (len(self.times) > 0):
            posDiff = newPosition - self.positions[-1]
            timeDiff = time - self.times[-1]
            vel = posDiff / timeDiff
            self.velocities.append(vel)
        else:
            self.velocities.append(0)

        self.times.append(time)
        self.positions.append(newPosition)
        self.update()

    def update(self):
        self.axs[0].cla()
        self.axs[0].plot(self.times, self.positions)
        self.axs[0].set_ylabel('Positions (rad)')
        
        self.axs[1].cla()
        self.axs[1].plot(self.times, self.velocities)
        self.axs[1].set_ylabel('Velocities (rad/s)')
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
