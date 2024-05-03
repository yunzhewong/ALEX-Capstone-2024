import time
import math
import sys
from typing import List
import aiosv2
import aios
import threading


def calculate_hertz(start, end):
    return 1 / (end - start)


def callCVP(motor: aiosv2.ConnectedMotor):
    print(motor.getCVP())


def bulkCVP(motor1: aiosv2.ConnectedMotor):
    BULK_COUNT = 100
    for _ in range(BULK_COUNT):
        callCVP(motor1)


def threaded(motors: List[aiosv2.ConnectedMotor]):
    threads = []

    start = time.perf_counter()

    for motor in motors:
        thread = threading.Thread(target=bulkCVP, args=(motor,))
        threads.append(thread)

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

    end = time.perf_counter()
    return calculate_hertz(start, end) * 100


if __name__ == "__main__":
    connected_addresses = aiosv2.get_addresses()
    connected_addresses.enable()

    connectedMotors = connected_addresses.getConnectedMotors()
    time.sleep(1)

    threaded(connectedMotors)

    connected_addresses.disable()
