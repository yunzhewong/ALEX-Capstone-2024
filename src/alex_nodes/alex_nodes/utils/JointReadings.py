from sensor_msgs.msg import JointState
import utils.ros as ros


class JointReadings:
    def __init__(self):
        self.last_time = 0
        self.time = 0
        self.velocities = []
        self.positions = []

        self.old_velocities = []

    def get_time(self):
        return self.time

    def get_iteration_time(self):
        return self.time - self.last_time

    def get_reading(self, index: int):
        if len(self.old_velocities) == 0:
            return self.positions[index], 0

        total_velocity = 0
        for old_reading in self.old_velocities:
            total_velocity += old_reading[index]

        average_velocity = total_velocity / len(self.old_velocities)
        return self.positions[index], average_velocity

    def set_readings(self, msg: JointState):
        if len(self.velocities) != 0:
            self.old_velocities.append(self.velocities[:])
            if len(self.old_velocities) > 20:
                self.old_velocities.pop(0)

        self.last_time = self.time
        self.time = ros.decode_time(msg)
        self.positions = msg.position
        self.velocities = msg.velocity
