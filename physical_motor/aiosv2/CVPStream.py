import threading
from typing import List
from aiosv2.AiosSocket import AiosSocket
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.ConnectedMotor import CVPConverter
from aiosv2.CVP import CVP


class CVPStream:
    def __init__(self, socket: AiosSocket, motors: List[SafeMotor]):
        self.socket = socket
        self.motors = motors
        self.stop = False
        self.readThread = None
        self.requestThread = None
        self.converter = CVPConverter()

    def start(self):
        self.readThread = threading.Thread(target=self.read)
        self.readThread.start()

        self.requestThread = threading.Thread(target=self.request)
        self.requestThread.start()

    def disable(self):
        self.stop = True
        self.readThread.join()
        self.requestThread.join()

    def read(self):
        while not self.stop:
            json_obj, ip = self.socket.readJSON()

            for motor in self.motors:
                if motor.getIP() == ip:
                    cvp = self.converter.parseCVP(json_obj)
                    motor.setCVP(cvp)
                    print(ip)

    def request(self, motor: SafeMotor):
        while not self.stop:
            for motor in self.motors:
                motor.requestCVP()
