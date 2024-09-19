from aiosv2.SafeMotorOperation import SafeMotor


EPSILON = 0.02
SLOW_VELOCITY = 0.1
def slow_move_to_pos(motor: SafeMotor, desiredPosition: float):
    move_to_pos(motor, desiredPosition, SLOW_VELOCITY)

def move_to_pos(motor: SafeMotor, desiredPosition: float, velocity: float):
    cvp = motor.getCVP()

    error = desiredPosition - cvp.position
    
    # the current position is less than desired position, increase current position
    if error > EPSILON:
        motor.setVelocity(velocity) 
    elif error < -EPSILON:
        motor.setVelocity(-velocity)
    else:
        motor.setVelocity(0)