from aiosv2.SafeMotorOperation import SafeMotor

CONTROLLER_P_GAIN = 10
CONTROLLER_I_GAIN = 0.1

class PIController():
    def __init__(self, Kp: float, Ki: float):
        self.Kp = Kp
        self.Ki = Ki
        self.integral_error = 0
    
    def compute_control(self, error: float, dt: float):
        self.integral_error += error * dt
        return self.Kp * error + self.Ki * self.integral_error  

class CascadeController():
    def __init__(self):
        self.position_controller: PIController = PIController(CONTROLLER_P_GAIN, CONTROLLER_I_GAIN)

    def set_reference(self, motor: SafeMotor, reference_position, reference_velocity, dt: float):
        cvp = motor.getCVP()

        position_error = reference_position - cvp.position
        position_correction = self.position_controller.compute_control(position_error, dt)
        motor_velocity = reference_velocity + position_correction

        motor.setVelocity(motor_velocity)
