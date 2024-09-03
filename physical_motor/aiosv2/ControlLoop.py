import time
from typing import Any, Callable

class MotorCombination:
    def __init__(self, no_robot=False):
        self.no_robot = no_robot  # Store the no_robot flag

    def enable(self):
        if not self.no_robot:
            # Add actual enable logic here
            pass

    def requestReadyCheck(self):
        if not self.no_robot:
            # Add actual ready check logic here
            pass

    def isReady(self):
        if self.no_robot:
            return True  # Automatically return True when NO_ROBOT is True
        # Otherwise, add actual isReady logic here
        return False

    def logCalibrationData(self):
        if not self.no_robot:
            # Add actual log calibration data logic here
            pass

    def getStreamError(self):
        if self.no_robot:
            return None  # Automatically return None when NO_ROBOT is True
        # Otherwise, add actual getStreamError logic here
        return "Some Error"

    def disable(self):
        if not self.no_robot:
            # Add actual disable logic here
            pass


# Experimentally, a sampling frequency of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY

def setup_teardown_motor_combination(
    combination: MotorCombination, actions: Callable[[Any, float], None], totalRunningTime: float, no_robot=False
):
    try:
        if not no_robot:
            print()

            combination.enable()
            print("Motors Enabled")

            combination.requestReadyCheck()

            while not combination.isReady():
                print("Checking Encoder Status...")
                time.sleep(0.1)
            print("Motors Ready")

            combination.logCalibrationData()

            result = input("Do you want to continue? (y): ")

            if result != 'y':
                raise Exception("Cancelled by the user")
    
        print("Starting Control Loop")

        startTime = time.perf_counter()
        currentTime = startTime
        endTime = currentTime + totalRunningTime
        lastLoop = currentTime
        averageLoopTime = 0
        samples = 0

        try:
            while currentTime < endTime:
                currentTime = time.perf_counter()

                loopTime = currentTime - lastLoop
                averageLoopTime = (averageLoopTime * samples + loopTime) / (samples + 1)
                samples += 1
                lastLoop = currentTime

                if not no_robot:
                    error = combination.getStreamError()
                    if error:
                        raise Exception(error)

                runningTime = currentTime - startTime
                actions(combination, runningTime)

                time.sleep(SAMPLING_PERIOD)
            print(f"Average Loop Time: {averageLoopTime:.4f}s")
            print("Control Loop Complete without Errors")
        except Exception as e:
            print(e)

        if not no_robot:
            combination.disable()
    except KeyboardInterrupt:
        print("Keyboard Interrupted: Motors notified to turn off")
        if not no_robot:
            combination.disable()
        print("Keyboard Interrupt again to release locks")

