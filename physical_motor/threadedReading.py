import time
import math
import sys
from typing import List
import aiosv2
import aios
import threading
import matplotlib.pyplot as plt

SAMPLING_FREQUENCY = 500


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
            start = time.perf_counter()
            end = start + (1 / SAMPLING_FREQUENCY)
            while start < end:
                start = time.perf_counter()

    def read(self):
        while not self.closed:
            try:
                json, ip = self.socket.readJSON()
                current = json.get("current")
                velocity = json.get("velocity")
                position = json.get("position")
                newCVP = aiosv2.CVP(current, velocity, position)
                with self.lock:
                    if self.data.get(ip) is None:
                        self.data[ip] = []

                    readTime = time.perf_counter()
                    self.data[ip].append((readTime, newCVP))
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
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()
    connectedMotors = [twinMotor.bottomMotor, twinMotor.topMotor]

    cvpReader = CVPReader(socket, connectedMotors)
    cvpReader.start()

    time.sleep(5)
    cvpReader.stop()

    firstTime = None

    for ip in cvpReader.data.keys():
        ipData = cvpReader.data[ip]
        firstTime = ipData[0][0]

        runningSum = 0
        minDiff = float("inf")
        maxDiff = 0

        for i in range(len(ipData) - 1):
            diff = ipData[i + 1][0] - ipData[i][0]
            runningSum += diff
            minDiff = min(diff, minDiff)
            maxDiff = max(diff, maxDiff)
        avgDiff = runningSum / len(ipData)

        avgVal = convertTimesToHz(avgDiff)
        minVal = convertTimesToHz(minDiff)
        maxVal = convertTimesToHz(maxDiff)
        print(f"IP: {ip}, Average: {avgVal}, Minimum: {minVal}, Maximum: {maxVal}")

        times = [ipData[i][0] for i in range(len(ipData))]

        plt.plot(times, label=f"Times for {ip}")
        print(times)

    typicalSampling = [
        firstTime + i * (1 / SAMPLING_FREQUENCY) for i in range(len(ipData))
    ]
    plt.plot(typicalSampling, label=f"Expected Sampling Times")
    plt.xlabel("Index")
    plt.ylabel("Times")
    plt.legend()
    plt.show()

    twinMotor.disable()
