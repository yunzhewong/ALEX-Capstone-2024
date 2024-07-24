
from enum import Enum
import math
import aiosv2
from classes.DataLog import DataLog
from aiosv2.TwinMotor import setup_teardown_twin_motor
from aiosv2 import CVP

class State(Enum):
    Collecting = 1,
    Paused = 2,
    Resetting = 3

MAX_ANGLE = 10 * math.pi
MAX_TIME = 10
STARTING_ANGLE = -10 * math.pi
RESET_TIME = 10
PAUSE_TIME = 1
START_MAGNITUDE = 0.3
INCREMENT_MAGNITUDE = 0.02


class BulkDataBatcher():

    def __init__(self):
        self.log = DataLog()

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
        if cvp is None:
            return
        
        position = cvp.position

        if self.state == State.Collecting:
            if not self.initialised:
                self.collecting_start = t
                self.initialised = True
                self.log = DataLog()

            current = START_MAGNITUDE + self.collect_index * INCREMENT_MAGNITUDE 

            self.log.addCVP(t, cvp, CVP(0,0,0))

            exceeded_max_angle = position > MAX_ANGLE
            exceeded_max_time = (t - self.collecting_start) >= MAX_TIME
            should_stop = exceeded_max_angle or exceeded_max_time

            if should_stop:
                self.log.download(f"step{format(round(current, 2), '.2f')}A.csv")
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
                self.reset_start = t
                self.reset_end = t + RESET_TIME
                self.reset_angle = position
                self.initialised = True

            expected_pos = self.reset_angle - ((self.reset_angle - STARTING_ANGLE) / RESET_TIME) * (t - self.reset_start)

            if t > self.reset_end:
                self.post_pause_state = State.Collecting
                self.state = State.Paused
                self.initialised = False
            
            connection.setPosition(expected_pos)

if __name__ == "__main__":
    dataBatcher = BulkDataBatcher()

    def func(twinMotor: aiosv2.TwinMotor, runningTime: float):
        dataBatcher.at_time(runningTime, twinMotor.topMotor)

    setup_teardown_twin_motor(func, 1000)

    
        
       
