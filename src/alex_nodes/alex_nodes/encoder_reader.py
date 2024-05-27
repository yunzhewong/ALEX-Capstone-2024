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

    def plot(self, times, ax1, ax2, ax3):
        ax1.cla()
        ax1.plot(times, self.currents, label="Current", color="blue")
        ax1.set_ylabel("Current")
        ax1.legend()

        ax2.cla()
        ax2.plot(times, self.velocities, label="Velocity", color="red")
        ax2.set_ylabel("Velocity")
        ax2.legend()

        ax3.cla()
        ax3.plot(times, self.positions, label="Position (rad)", color="green")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Position")
        ax3.legend()

    def export(self):
        npCurrents = np.array(self.currents)
        npVelocities = np.array(self.velocities)
        npPositions = np.array(self.positions)
        return (npCurrents, npVelocities, npPositions)
        

class StateReader(Node):
    def __init__(self):
        super().__init__("encoder_reader")
        _, axs = plt.subplots(3, 2)
        self.axs = axs
        
        self.position_sub = self.create_subscription(
            JointState, "/joint_states", self.read_position, 10
        )
        self.bottomTorque = self.create_subscription(
                Float64MultiArray, f"/bottom_motor_controller/commands", lambda msg: self.read_torque(msg, 0), 10
            )
        self.topTorque = self.create_subscription(
                Float64MultiArray, f"/top_motor_controller/commands", lambda msg: self.read_torque(msg, 1), 10
            )
        self.times = []
        self.dataStores = [DataStore(), DataStore()]
        self.count = 0

    def read_torque(self, msg: Float64MultiArray, index):
        self.dataStores[index].setTorque(msg.data[0])

    def read_position(self, msg: JointState):
        timestamp = msg.header.stamp

        time = timestamp.sec + timestamp.nanosec / 1e9

        self.times.append(time)
        for i in range(len(self.dataStores)):
            newPosition = msg.position[i]
            newVelocity = msg.velocity[i]
            self.dataStores[i].addPositionVelocity(newPosition, newVelocity)

    def export_data(self):
        npTimes = np.array(self.times)
        totalTuple = (npTimes, )
        for dataStore in self.dataStores:
            exportTuple = dataStore.export()
            totalTuple += exportTuple
        return np.column_stack(totalTuple)

def main(args=None):
    rclpy.init(args=args)

    state_reader = StateReader()

    try:
        rclpy.spin(state_reader)
    except KeyboardInterrupt:
        data = state_reader.export_data()
        np.savetxt('data.csv', data, delimiter=',', header='times(s), bottom currents(A), bottom velocities(rads^-1), bottom positions(rads), top currents(A), top velocities(rads^-1), top positions(rads)', comments="")

    state_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
