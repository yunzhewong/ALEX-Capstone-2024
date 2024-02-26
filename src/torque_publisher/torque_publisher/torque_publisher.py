import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

KNEE_CONTROLLER_NAME = 'knee_joint_controller'
HIP_CONTROLLER_NAME = 'hip_joint_controller'

class TorquePublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.hip_publisher = self.create_publisher(Float64MultiArray, f'/{HIP_CONTROLLER_NAME}/commands', 10)
        self.knee_publisher = self.create_publisher(Float64MultiArray, f'/{KNEE_CONTROLLER_NAME}/commands', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.i = -200

    def publish_message(self):
        msg = Float64MultiArray()
        msg.data = [float(self.i)]
        msg.layout.data_offset = 0
        self.hip_publisher.publish(msg)
        self.knee_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 40


def main(args=None):
    rclpy.init(args=args)

    torque_publisher = TorquePublisher()

    rclpy.spin(torque_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    torque_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()