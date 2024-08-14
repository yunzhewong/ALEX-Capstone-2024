import math

from matplotlib import pyplot as plt
import numpy as np
from aiosv2.CVP import CVP
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from classes.DataLog import CVPPlot
from aiosv2.TwinMotor import TwinMotor, setup_teardown_twin_motor
from functions.TrajectoryGeneration import findCubicSpline, evaluateCubicSpline

        
class State():
    def __init__(self):
        self.log = CVPPlot()
        pass



    
if __name__ == "__main__":

    coefficients = findCubicSpline(0, 10, 0, math.pi, 0, 0)
    expected_velocities = []
    expected_positions = []

    K_P = 10

    state = State()
    def func(twinMotor: TwinMotor, runningTime: float):

        cvp = twinMotor.bottomMotor.getCVP()

        state.log.addCVP(runningTime, cvp)

        referencePosition, referenceVelocity = evaluateCubicSpline(coefficients, runningTime)
        expected_velocities.append(referenceVelocity)
        expected_positions.append(referencePosition)

        positionError = referencePosition - cvp.position

        velocityCommand = referenceVelocity + K_P * positionError

        twinMotor.bottomMotor.setVelocity(velocityCommand)



    setup_teardown_twin_motor(func, 10)



    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    times = np.array(state.log.times)



    ax1.plot(times, np.array(state.log.velocities), label="True", color="blue")
    ax1.plot(times, np.array(expected_velocities), label="Expected", color="red")
    ax1.set_ylabel("Velocity (rad/s)")
    ax1.legend()

    ax2.plot(times, np.array(state.log.positions), label="True", color="blue")
    ax2.plot(times, np.array(expected_positions), label="Expected", color="red")
    ax2.set_ylabel("Positions (rad)")
    ax2.legend()
    

    plt.tight_layout()

    plt.show()
    