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
    motor.getCVP()


def time_single(motor: aiosv2.ConnectedMotor):
    start = time.perf_counter()

    callCVP(motor)

    end = time.perf_counter()

    return calculate_hertz(start, end)


def time_double(motor1: aiosv2.ConnectedMotor, motor2: aiosv2.ConnectedMotor):
    start = time.perf_counter()

    callCVP(motor1)
    callCVP(motor2)

    end = time.perf_counter()

    return calculate_hertz(start, end)


def time_single_bulk(motor1: aiosv2.ConnectedMotor):
    start = time.perf_counter()

    BULK_COUNT = 100
    for _ in range(BULK_COUNT):
        callCVP(motor1)

    end = time.perf_counter()

    print(motor1.ip, calculate_hertz(start, end) * BULK_COUNT)

    return calculate_hertz(start, end) * BULK_COUNT


def time_double_bulk(motor1: aiosv2.ConnectedMotor, motor2: aiosv2.ConnectedMotor):
    start = time.perf_counter()

    BULK_COUNT = 100
    for _ in range(BULK_COUNT):
        callCVP(motor1)
        callCVP(motor2)

    end = time.perf_counter()

    return calculate_hertz(start, end) * BULK_COUNT


def time_multi(motors: List[aiosv2.ConnectedMotor]):
    start = time.perf_counter()

    BULK_COUNT = 100
    for _ in range(BULK_COUNT):
        for motor in motors:
            callCVP(motor)

    end = time.perf_counter()

    return calculate_hertz(start, end) * BULK_COUNT


def threaded(motors: List[aiosv2.ConnectedMotor]):
    threads = []

    start = time.perf_counter()

    for motor in motors:
        thread = threading.Thread(target=time_single_bulk, args=(motor,))
        threads.append(thread)

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

    end = time.perf_counter()
    return calculate_hertz(start, end) * 100


if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor()
    twinMotor.enable()

    time.sleep(1)

    bottomMotor = twinMotor.bottomMotor
    topMotor = twinMotor.topMotor

    print(time_single(bottomMotor))
    print(time_single_bulk(bottomMotor))
    print(time_double(bottomMotor, topMotor))
    print(time_double_bulk(bottomMotor, topMotor))
    print(time_multi([bottomMotor, topMotor]))
    print(threaded([bottomMotor, topMotor]))

    twinMotor.disable()
