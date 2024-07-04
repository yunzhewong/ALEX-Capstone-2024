
from enum import Enum
import math
import aiosv2
from classes.DataLog import DataLog
from aiosv2.TwinMotor import setup_teardown_twin_motor

class State(Enum):
    Collecting = 1,
    Paused = 2,
    Resetting = 3

MAX_ANGLE = math.pi / 2
MAX_TIME = 5
STARTING_ANGLE = -math.pi / 2
RESET_TIME = 5
PAUSE_TIME = 1
LOG_FREQUENCY = 1
LOG_INCREMENT = 0.1


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
        print(self.state)

        if self.state == State.Collecting:
            if not self.initialised:
                self.collecting_start = t
                self.initialised = True
                self.log = DataLog()

            frequency = 10 ** (LOG_FREQUENCY + LOG_INCREMENT * self.collect_index) 

            self.log.addCVP(t, cvp)

            # exceeded_max_angle = position > MAX_ANGLE
            exceeded_max_time = (t - self.collecting_start) >= MAX_TIME
            # should_stop = exceeded_max_angle or exceeded_max_time
            command = 5 * math.sin(2 * math.pi * frequency * (t - self.collecting_start))

            if exceeded_max_time:
                self.log.download(f"sinusoid{round(frequency, 2)}Hz.csv")
                self.state = State.Paused
                self.post_pause_state = State.Resetting
                self.collect_index += 1
                self.initialised = False

            connection.setCurrent(command)
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
        dataBatcher.at_time(runningTime, twinMotor.bottomMotor)

    setup_teardown_twin_motor(func, 1000)

    
        
       
