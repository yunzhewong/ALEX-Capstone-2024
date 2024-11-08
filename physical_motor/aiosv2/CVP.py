# Represents the Current, Velocity and Position of the motor
# Has attributes for current, velocity and position.
class CVP:
    def __init__(self, current: float, velocity: float, position: float):
        self.current = current
        self.velocity = velocity
        self.position = position

    def __str__(self):  # provide a string representation of the object
        return f"Current: {self.current:8.4f}, Velocity: {self.velocity:8.4f}, Position: {self.position:8.4f}"
