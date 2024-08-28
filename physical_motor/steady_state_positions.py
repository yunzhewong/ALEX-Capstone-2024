from enum import Enum
import math
from typing import List
from aiosv2.CSVWriter import CSVWriter
import trajectory
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.CVP import CVP
from classes.DataLog import CVPPlot


def stop_collection(cvp: CVP):
    return cvp.position > (math.pi / 2)


HOLD_TIME = 5

START_MAGNITUDE = 0.8
INCREMENT_MAGNITUDE = 0.05
END_MAGNITUDE = 2

COUNT = int((END_MAGNITUDE - START_MAGNITUDE) / INCREMENT_MAGNITUDE) + 2
currents = [START_MAGNITUDE + INCREMENT_MAGNITUDE * i for i in range(COUNT)]

class BulkDataBatcher:
    def __init__(self, collectionPoints: List[float]):
        self.writer: CSVWriter | None = None
        self.cvpPlot = CVPPlot()
        self.collectionPoints = collectionPoints

    def at_time(self, t: float, connection: SafeMotor):
        if self.writer is None:
            self.writer = CSVWriter("increasing_rightknee", [connection])

        collectionIndex = math.floor(t / HOLD_TIME)
        current = self.collectionPoints[collectionIndex]
        print(current)

        connection.setCurrent(current)
        self.cvpPlot.addCVP(t, connection.getCVP())
        self.writer.addCVP(t, [connection])

if __name__ == "__main__":
    print(currents)
    dataBatcher = BulkDataBatcher(currents)

    def func(exoskeleton: Exoskeleton, runningTime: float):
        trajectory.slow_move_to_pos(exoskeleton.rightAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.rightExtensor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftExtensor, 0)
        dataBatcher.at_time(runningTime, exoskeleton.rightKnee)
        trajectory.slow_move_to_pos(exoskeleton.leftKnee, 0)


    setup_teardown_motor_combination(Exoskeleton(), func, 1000)
    dataBatcher.cvpPlot.plot()
    
