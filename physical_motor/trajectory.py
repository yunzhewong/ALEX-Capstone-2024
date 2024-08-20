from aiosv2.SafeMotorOperation import SafeMotor


EPSILON = 0.05
SLOW_VELOCITY = 0.1
def slow_move_to_pos(motor: SafeMotor, desiredPosition: float):
    cvp = motor.getCVP()

    error = desiredPosition - cvp.position
    
    # the current position is less than desired position, increase current position
    if error > EPSILON:
        motor.setVelocity(SLOW_VELOCITY) 
    elif error < -EPSILON:
        motor.setVelocity(-SLOW_VELOCITY)
    else:
        motor.setVelocity(0)
