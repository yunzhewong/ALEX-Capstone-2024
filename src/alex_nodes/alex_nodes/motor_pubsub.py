import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

PROPELLOR_CONTROLLER = "propellor_controller"
FREQUENCY = 100

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.publisher = self.create_publisher(
            Float64MultiArray, f"/{PROPELLOR_CONTROLLER}/commands", 10
        )
        
        timer_period = 1 / FREQUENCY  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)

        self.reader = self.create_subscription(
            JointState, "/joint_states", self.read_encoder, 10
        )
        self.position = None
        self.counter = 0

    def calculate_torque(self):
        if (self.position is None):
            return 0
        return -3 * self.position

    def publish_message(self):
        torque = self.calculate_torque()

        msg = Float64MultiArray()
        msg.data = [float(torque)]
        msg.layout.data_offset = 0
        self.controller.publish(msg)

    def read_encoder(self, msg: JointState):
        self.position = msg.position[0]


def main(args=None):
    rclpy.init(args=args)

    torque_publisher = MotorController()

    rclpy.spin(torque_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    torque_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
