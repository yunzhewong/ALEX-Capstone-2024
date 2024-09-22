class PIController:
    def __init__(self, Kp: float, Ki: float):
        self.Kp = Kp
        self.Ki = Ki
        self.integral_error = 0

    def compute_control(self, error: float, dt: float):
        self.integral_error += error * dt
        return self.Kp * error + self.Ki * self.integral_error


CONTROLLER_P_GAIN = 10
CONTROLLER_I_GAIN = 0.1


class CascadeController:
    def __init__(self):
        self.outer_controller = PIController(CONTROLLER_P_GAIN, CONTROLLER_I_GAIN)

    def calculate_velocity(
        self,
        position: float,
        reference_position: float,
        reference_velocity: float,
        dt: float,
    ):
        position_error = reference_position - position
        position_correction = self.outer_controller.compute_control(position_error, dt)
        print(position, position_correction)
        return reference_velocity + position_correction
