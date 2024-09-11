from enum import Enum
import math
from typing import List
from aiosv2.CSVWriter import CSVWriter
import trajectory
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.ControlLoop import setup_teardown_motor_combination
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.CVP import CVP

MAX_TIME = 20
PAUSE_TIME = 1.5

START_MAGNITUDE = 1
INCREMENT_MAGNITUDE = 0.01
END_MAGNITUDE = 1.5

COUNT = int((END_MAGNITUDE - START_MAGNITUDE) / INCREMENT_MAGNITUDE) + 2
currents = [START_MAGNITUDE + INCREMENT_MAGNITUDE * i for i in range(COUNT)]

class State(Enum):
    Collecting = 1
    Paused = 2
    Resetting = 3


class BulkDataBatcher:
    def __init__(self, collectionPoints: List[float]):
        self.log: CSVWriter | None = None

        self.state = State.Resetting
        self.post_pause_state = State.Paused
        self.initialised = False
        self.collectionPoints = collectionPoints
        self.collecting_start = -1
        self.collect_index = 0
        self.pause_end_time = -1
        self.reset_start = -1
        self.reset_end = -1
        self.reset_angle = -1
        self.initial_angle = -1

    def at_time(self, t, connection: SafeMotor):
        cvp = connection.getCVP()

        if self.state == State.Collecting:
            current = self.collectionPoints[self.collect_index]

            if not self.initialised:
                print(current)
                self.collecting_start = t
                self.initialised = True
                self.log = CSVWriter(
                    f"current{format(round(current, 2), '.2f')}A.csv", [connection]
                )

            self.log.addCVP(t, [connection])

            if (t - self.collecting_start) >= MAX_TIME:
                self.log.close()
                print(cvp)
                self.state = State.Paused
                self.post_pause_state = State.Resetting
                self.collect_index += 1
                self.initialised = False

            connection.setCurrent(current)
        elif self.state == State.Paused:
            if not self.initialised:
                self.pause_end_time = t + PAUSE_TIME
                self.initialised = True

            if t > self.pause_end_time:
                self.state = self.post_pause_state
                self.initialised = False

            connection.setCurrent(0)
        else:
            if not self.initialised:
                if self.initial_angle == -1:
                    self.initial_angle = cvp.position
                    print(self.initial_angle)
                self.initialised = True

            trajectory.slow_move_to_pos(connection, self.initial_angle)
            print(cvp.position - self.initial_angle)
            if abs(cvp.position - self.initial_angle) < 0.02:
                self.post_pause_state = State.Collecting
                self.state = State.Paused
                self.initialised = False


if __name__ == "__main__":

    dataBatcher = BulkDataBatcher(currents)

    def func(exoskeleton: Exoskeleton, runningTime: float):
        dataBatcher.at_time(runningTime, exoskeleton.rightAbductor)
        trajectory.slow_move_to_pos(exoskeleton.leftAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftExtensor, 0)
        trajectory.slow_move_to_pos(exoskeleton.rightExtensor, 0)
        trajectory.slow_move_to_pos(exoskeleton.rightKnee, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftKnee, 0)


    setup_teardown_motor_combination(Exoskeleton(), func, 1000)
    
