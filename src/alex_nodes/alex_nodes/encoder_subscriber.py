import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np

NUMBER_OF_JOINTS = 3

class EncoderSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.times = []
        self.data = [[] for i in range(NUMBER_OF_JOINTS)]
        self.colors = ["blue", "red", "green"]

    def listener_callback(self, msg: JointState):
        timestamp = msg.header.stamp
        time = timestamp.sec + timestamp.nanosec * 1e-9
        self.times.append(time)
        for i in range(NUMBER_OF_JOINTS):
            self.data[i].append(msg.position[i])
        

        if (len(self.times) > 50):
            self.times.pop(0)
            for i in range(NUMBER_OF_JOINTS):
                self.data[i].pop(0)
          
        plt.clf()
        for i in range(NUMBER_OF_JOINTS):
            plt.plot(self.times, self.data[i], linewidth=2.0, color=self.colors[i], label=msg.name[i])
        plt.legend(loc='upper right')
        plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)

    encoder_subscriber = EncoderSubscriber()

    rclpy.spin(encoder_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()