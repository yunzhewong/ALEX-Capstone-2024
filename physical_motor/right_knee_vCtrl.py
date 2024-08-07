import math
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import CVPPlot

URATION = 10

SAMPLE_PERIOD = 0.04
POLYNOMIAL_COEFFICIENTS = [
    0.0,
    0.0,
    (3 * math.pi / 200),
    (-1 * math.pi / 1000),
]  # Coefficients for the position reference

VEL_COEFFICIENTS = [
    0.0,
    2*(3 * math.pi / 200),
    3*(-1 * math.pi / 1000),
]  # Coefficients for the velocity reference
        
DESIRED_TRAJECTORY = [0.0, 0.0, (3 * math.pi / 200), (-1 * math.pi / 1000)]
DESIRED_VELOCITY = [0.0, 2*(3 * math.pi / 200), 3*(-1 * math.pi / 1000)]

# PID controller parameters
Kp = 1.0
Ki = 0.1
Kd = 0.05

# PID controller state
integral_error = 0.0
previous_error = 0.0

def calculate_refPosition(running_time: float):
    return sum(
        coef * (running_time**i)
        for i, coef in enumerate(POLYNOMIAL_COEFFICIENTS)
    )


def calculate_refVelocity(running_time: float):
    return sum(
        coef * (running_time**i) for i, coef in enumerate(VEL_COEFFICIENTS)
    )


def pid_controller(error, integral_error, previous_error, dt):
    P = Kp * error
    I = Ki * integral_error
    D = Kd * (error - previous_error) / dt
    return P + I + D

class State():
    def __init__(self):
        self.log = CVPPlot()
        pass

    
if __name__ == "__main__":

    state = State()
    def func(exoMotor: RightKneeExoMotor, runningTime: float):
        global integral_error, previous_error

        # Calculate reference position bottom
        ref_position = calculate_refPosition(runningTime)

        # Get current positions
        current_position = (
            RightKneeExoMotor.getCVP().position if RightKneeExoMotor.getCVP() is not None else 0.0
        )

        # Calculate errors
        error = ref_position - current_position

        # Update integral and derivative errors
        integral_error += error * SAMPLE_PERIOD

        # Calculate control outputs
        velocity = pid_controller(
            error, integral_error, previous_error, SAMPLE_PERIOD
        ) + calculate_refVelocity(runningTime)

        # Incorporate desired trajectory and velocity
        desired_position = sum(
            coef * (runningTime**i) for i, coef in enumerate(DESIRED_TRAJECTORY)
        )
        desired_velocity = sum(
            coef * (runningTime**i) for i, coef in enumerate(DESIRED_VELOCITY)
        )

        # Adjust control outputs based on desired values
        velocity += desired_velocity - calculate_refVelocity(runningTime)
        # Update motors' velocity
        RightKneeExoMotor.setVelocity(velocity)

        # Update previous errors
        previous_error = error

        # Plotting and saving data
        dataLog = CVPPlot(
        desired_trajectory_bottom=DESIRED_TRAJECTORY,
        desired_velocity_bottom=DESIRED_VELOCITY,
        )


        # exoMotor.motor.setVelocity(-2)

        # cvp = exoMotor.motor.getCVP()
        # state.log.addCVP(runningTime, cvp)


    setup_teardown_rightknee_exomotor(func, 10)
    state.log.plot()
    