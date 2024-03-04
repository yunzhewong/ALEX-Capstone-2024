import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np


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
        self.data = [[], [], []]
        self.colors = ["blue", "red", "green"]
        self.count = 0

    def listener_callback(self, msg: JointState):
        self.times.append(self.count)
        for i in range(3):
            self.data[i].append(msg.position[i])
        
        self.count += 1

        if (len(self.times) > 50):
            self.times.pop(0)
            for i in range(3):
                self.data[i].pop(0)
          
        plt.clf()
        for i in range(3):
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