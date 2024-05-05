import time
import math
import sys
from typing import List
import aiosv2
import aios
import threading


class CVPReader:
    def __init__(self, socket: aiosv2.AiosSocket, motors: List[aiosv2.ConnectedMotor]):
        self.lock = threading.Lock()
        self.socket = socket
        self.motors = motors
        self.closed = False

        self.readThread = threading.Thread(target=self.read)
        self.requestThread = threading.Thread(target=self.request)
        self.data = {}

    def start(self):
        self.readThread.start()
        self.requestThread.start()

    def request(self):
        for _ in range(100):
            for motor in self.motors:
                motor.requestCVP()
            time.sleep(0.01)

    def read(self):
        while not self.closed:
            try:
                json, ip = self.socket.readJSON()
                with self.lock:
                    if self.data.get(ip) is None:
                        self.data[ip] = []
                    readTime = time.perf_counter()
                    self.data[ip].append(readTime)
            except Exception as e:
                print(e)

    def stop(self):
        self.closed = True
        self.readThread.join()
        self.requestThread.join()


def convertTimesToHz(diff):
    return 1 / diff


if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    connected_addresses = socket.get_addresses()
    connected_addresses.enable()
    connectedMotors = connected_addresses.getConnectedMotors()

    cvpReader = CVPReader()
    cvpReader.start()

    cvpReader.stop()

    for ip in cvpReader.data.keys():
        times = cvpReader.data[ip]

        runningSum = 0
        minDiff = float("inf")
        maxDiff = 0
        for i in range(len(times) - 1):
            diff = times[i + 1] - times[i]
            runningSum += diff
            minDiff = min(diff, minDiff)
            maxDiff = max(diff, maxDiff)
        avgDiff = runningSum / len(times)

        avg = convertTimesToHz(avgDiff)
        min = convertTimesToHz(minDiff)
        max = convertTimesToHz(maxDiff)
        print(f"IP: {ip}, Average: {avg}, Minimum: {min}, Maximum: {max}")

    connected_addresses.disable()
