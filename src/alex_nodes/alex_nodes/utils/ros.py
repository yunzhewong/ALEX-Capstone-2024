from sensor_msgs.msg import JointState


def decode_time(jointState: JointState):
    timestamp = jointState.header.stamp
    return timestamp.sec + timestamp.nanosec / 1e9
