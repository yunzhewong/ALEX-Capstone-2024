from enum import Enum
import math
from typing import List
from aiosv2.CSVWriter import CSVWriter
import aiosv2
from classes.DataLog import DataLog
from aiosv2.RightKneeExoMotor import (
    RightKneeExoMotor,
    setup_teardown_rightknee_exomotor,
)
from aiosv2.TwinMotor import setup_teardown_twin_motor
from aiosv2 import CVP
import trajectory


class State(Enum):
    Collecting = (1,)
    Paused = (2,)
    Resetting = 3

MAX_TIME = 5
PAUSE_TIME = 1

START_MAGNITUDE = 0
INCREMENT_MAGNITUDE = 0.05
END_MAGNITUDE = 3.00

COUNT = int((END_MAGNITUDE - START_MAGNITUDE) / INCREMENT_MAGNITUDE) + 1
magnitudes = [START_MAGNITUDE + INCREMENT_MAGNITUDE * i for i in range(COUNT)]


class BulkDataBatcher:
    def __init__(self,  magnitudes: List[float]):
        self.log: CSVWriter | None = None
        self.magnitudes = magnitudes

        self.state = State.Resetting
        self.post_pause_state = State.Paused
        self.initialised = False
        self.collecting_start = -1
        self.collect_index = 0
        self.pause_end_time = -1
        self.reset_start = -1
        self.reset_end = -1
        self.reset_angle = -1

    def at_time(self, t, connection: aiosv2.SafeMotor):
        cvp = connection.getCVP()

        if self.state == State.Collecting:
            magnitude = self.magnitudes[self.collect_index]

            if not self.initialised:
                print(magnitude)
                self.collecting_start = t
                self.initialised = True
                self.log = CSVWriter(
                    f"current{format(round(magnitude, 2), '.2f')}A.csv", [connection]
                )

            self.log.addCVP(t, [connection])

            if (t - self.collecting_start) >= MAX_TIME:
                self.log.close()
                print(cvp)
                self.state = State.Paused
                self.post_pause_state = State.Resetting
                self.collect_index += 1
                self.initialised = False

            connection.setCurrent(magnitude)
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
                self.initialised = True

            connection.setVelocity(-1)
            
            if cvp.position < 0.05:
                self.post_pause_state = State.Collecting
                self.state = State.Paused
                self.initialised = False


if __name__ == "__main__":
    dataBatcher = BulkDataBatcher(magnitudes)

    def func(exoMotor: RightKneeExoMotor, runningTime: float):
        dataBatcher.at_time(runningTime, exoMotor.motor)

    setup_teardown_rightknee_exomotor(func, 2000)
