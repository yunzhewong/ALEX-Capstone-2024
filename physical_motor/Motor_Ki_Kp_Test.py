import math
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.CSVWriter import CSVWriter
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import CVPPlot
from aiosv2 import aios
import trajectory

Kp_pos = 1
Kp_vel = 0.5
Ki_vel = 0.1
SAVE_NAME = f'Vel_C Kp_pos={Kp_pos}, Kp_vel={Kp_vel}, Ki_vel={Ki_vel}' + '.csv'
        
class State():
    def __init__(self):
        self.log = CVPPlot()
        self.csvwriter: CSVWriter | None = None
        self.initialised = False

def reset_position(rightKnee: RightKneeExoMotor, runningTime: float):
    if rightKnee.motor.getCVP().position > 0.1:
        rightKnee.motor.setVelocity(-0.5)
    elif rightKnee.motor.getCVP().position < -0.1:
        rightKnee.motor.setVelocity(0.5)
    else:
        rightKnee.motor.setPosition(0)

if __name__ == "__main__":
    
    state = State()

    def func(rightKnee: RightKneeExoMotor, runningTime: float):
        if not state.initialised:
            state.csvwriter = CSVWriter(SAVE_NAME, [rightKnee.motor])
            state.initialised = True

        cvp = rightKnee.motor.getCVP()

        rightKnee.motor.setVelocity(1*math.sin(runningTime))

        state.log.addCVP(runningTime, cvp)
        state.csvwriter.addCVP(runningTime, [rightKnee.motor])

    setup_teardown_motor_combination(RightKneeExoMotor(), func, 7)
    state.log.download(SAVE_NAME)
    setup_teardown_motor_combination(RightKneeExoMotor(), reset_position, 3)

    

    # state.log.plot()


    