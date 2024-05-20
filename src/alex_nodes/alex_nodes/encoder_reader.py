import os

from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from alex_interfaces.srv import Command
import numpy as np
from std_msgs.msg import Float64MultiArray
import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from motor_pubsub_utils.constants import MOTOR_TORQUE_CONSTANT

class DataStore():
    def __init__(self):
        self.currents = []
        self.velocities = []
        self.positions = []
        self.torque = 0

    def addPositionVelocity(self, position, velocity):
        self.positions.append(position)
        self.velocities.append(velocity)
        self.currents.append(self.torque / MOTOR_TORQUE_CONSTANT),

    def setTorque(self, torque):
        self.torque = torque
        

class StateReader(Node):
    def __init__(self):
        super().__init__("encoder_reader")
        _, axs = plt.subplots(3, 1)
        self.axs = axs
        
        self.position_sub = self.create_subscription(
            JointState, "/joint_states", self.read_position, 10
        )
        self.torque_sub = self.create_subscription(
                Float64MultiArray, f"/bottom_motor_controller/commands", self.read_torque, 10
            )
        self.times = []
        self.dataStore = DataStore()
        self.update()

    def read_torque(self, msg: Float64MultiArray):
        self.dataStore.setTorque(msg.data[0])

    def read_position(self, msg: JointState):
        timestamp = msg.header.stamp
        time = timestamp.sec + timestamp.nanosec / 1e9

        newPosition = msg.position[0]
        newVelocity = msg.velocity[0]
        self.times.append(time)
        self.dataStore.addPositionVelocity(newPosition, newVelocity)
        self.update()

    def update(self):
        self.axs[0].cla()
        self.axs[0].plot(self.times, self.dataStore.positions)
        self.axs[0].set_ylabel('Positions (rad)')
        
        self.axs[1].cla()
        self.axs[1].plot(self.times, self.dataStore.velocities)
        self.axs[1].set_ylabel('Velocities (rad/s)')

        self.axs[2].cla()
        self.axs[2].plot(self.times, self.dataStore.currents)
        self.axs[2].set_ylabel('Currents (A)')
        plt.pause(0.01)

    def export_data(self):
        npTimes = np.array(self.times)
        npPositions = np.array(self.dataStore.positions)
        npVelocities = np.array(self.dataStore.velocities)
        return np.column_stack((npTimes, npPositions, npVelocities))

def main(args=None):
    rclpy.init(args=args)

    state_reader = StateReader()

    try:
        rclpy.spin(state_reader)
    except KeyboardInterrupt:
        data = state_reader.export_data()
        np.savetxt('data.csv', data, delimiter=',', header='times(s), positions(rads), velocities(rads^-1)', comments="")

    state_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
