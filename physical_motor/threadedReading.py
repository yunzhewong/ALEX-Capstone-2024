import time
import math
import sys
from typing import List
import aiosv2
import aios
import threading


def getCVP(motor: aiosv2.ConnectedMotor):
    return motor.getCVP()


def readSocket(socket: aiosv2.AiosSocket):
    count = 0
    while True:
        try:
            json = socket.readJSON()
            count += 1
        except Exception as e:
            print(e)
            print(count)


def requestCVP(motor: aiosv2.ConnectedMotor):
    for _ in range(100):
        motor.requestCVP()


if __name__ == "__main__":
    socket = aiosv2.AiosSocket()

    connected_addresses = socket.get_addresses()
    connected_addresses.enable()

    connectedMotors = connected_addresses.getConnectedMotors()
    time.sleep(1)

    readThread = threading.Thread(target=readSocket, args=(socket,))
    readThread.start()

    requestThread = threading.Thread(target=requestCVP, args=(connectedMotors[0],))
    requestThread.start()

    connected_addresses.disable()
