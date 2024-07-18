from rclpy.qos import QoSReliabilityPolicy, QoSProfile

BestEffortQoS = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)