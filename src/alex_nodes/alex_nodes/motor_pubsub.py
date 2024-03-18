import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

PROPELLOR_CONTROLLER = "propellor_controller"
ENCODER_READER = "propellor_joint"


class TorquePublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.controller = self.create_publisher(
            Float64MultiArray, f"/{PROPELLOR_CONTROLLER}/commands", 10
        )
        self.reader = self.create_subscription(
            JointState, "/joint_states", self.callback, 10
        )
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.i = -10
        self.counter = 0

    def publish_message(self):
        msg = Float64MultiArray()
        msg.data = [float(self.i)]
        msg.layout.data_offset = 0
        self.controller.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def callback(self, msg: JointState):
        self.counter += 1

        if self.counter % 100 == 0:
            print(msg)
            self.get_logger().info("Received: %s" % msg)
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)

    torque_publisher = TorquePublisher()

    rclpy.spin(torque_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    torque_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
