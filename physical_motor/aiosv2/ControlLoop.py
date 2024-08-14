
import time
from typing import Any, Callable

class MotorCombination():
    def enable(self):
        pass

    def verifyReady(self):
        pass
    
    def logCalibrationData(self):
        pass
    
    def getStreamError(self):
        pass

    def disable(self):
        pass


# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY

def setup_teardown_motor_combination(
    combination: MotorCombination, actions: Callable[[Any, float], None], totalRunningTime: float
):
    try:
        combination.enable()

        print("Motor Combination Enabled")

        combination.verifyReady()

        combination.logCalibrationData()

        result = input("Do you want to continue? (y): ")

        if result != 'y':
            raise Exception("Cancelled by the user")
    

        startTime = time.perf_counter()
        currentTime = startTime
        endTime = currentTime + totalRunningTime

        try:
            while currentTime < endTime:
                currentTime = time.perf_counter()
                error = combination.getStreamError()
                if error:
                    raise Exception(error)

                runningTime = currentTime - startTime
                actions(combination, runningTime)

                time.sleep(SAMPLING_PERIOD)
        except Exception as e:
            print(e)

        combination.disable()
    except KeyboardInterrupt:
        print("Keyboard Interrupted: Motors turned off")
        print("Keyboard Interrupt again to release locks")
        combination.disable()