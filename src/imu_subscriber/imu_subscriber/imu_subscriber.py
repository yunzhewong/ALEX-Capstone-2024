import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np


class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.times = []
        self.data = []
        self.i = 0

    def listener_callback(self, msg: Imu):
        self.get_logger().info('I heard: "%s"' % msg.angular_velocity)
        self.get_logger().info('I heard: "%s"' % msg.linear_acceleration)

        self.times.append(self.i)
        self.data.append(msg.linear_acceleration.z)

        if (len(self.times) > 50):
            self.times.pop(0)
            self.data.pop(0)
          
        plt.clf()

        plt.plot(self.times, self.data, linewidth=2.0, color="blue")

        self.i += 1
        plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = IMUSubscriber()

    rclpy.spin(imu_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()