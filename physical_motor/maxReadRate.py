import time
import math
import sys
import aiosv2


def calculate_hertz(start, end):
    return 1 / (end - start)


def time_single(motor: aiosv2.ConnectedMotor):
    start = time.perf_counter()

    motor.getCVP()

    end = time.perf_counter()

    return calculate_hertz(start, end)


def time_double(motor1: aiosv2.ConnectedMotor, motor2: aiosv2.ConnectedMotor):
    start = time.perf_counter()

    motor1.getCVP()
    motor2.getCVP()

    end = time.perf_counter()

    return calculate_hertz(start, end)


def time_single_bulk(motor1: aiosv2.ConnectedMotor):
    start = time.perf_counter()

    BULK_COUNT = 100
    for _ in range(BULK_COUNT):
        motor1.getCVP()

    end = time.perf_counter()

    return calculate_hertz(start, end) * BULK_COUNT


def time_double_bulk(motor1: aiosv2.ConnectedMotor, motor2: aiosv2.ConnectedMotor):
    start = time.perf_counter()

    BULK_COUNT = 100
    for _ in range(BULK_COUNT):
        motor1.getCVP()
        motor2.getCVP()

    end = time.perf_counter()

    return calculate_hertz(start, end) * BULK_COUNT


if __name__ == "__main__":
    connected_addresses = aiosv2.get_addresses()
    connected_addresses.enable()

    connectedMotors = connected_addresses.getConnectedMotors()

    print("Maximum Sampling Frequency")

    single_hertz = time_single(connectedMotors[0])
    print(f"Single Read: {single_hertz} Hz")

    double_hertz = time_double(connectedMotors[0], connectedMotors[1])
    print(f"Double Read: {double_hertz} Hz")

    single_bulk_hertz = time_double(connectedMotors[0], connectedMotors[1])
    print(f"Bulk Single Read: Average {single_bulk_hertz} Hz")

    double_bulk_hertz = time_double(connectedMotors[0], connectedMotors[1])
    print(f"Bulk Double Read: Average {double_bulk_hertz} Hz")

    time.sleep(1)
    connected_addresses.disable()
