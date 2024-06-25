import threading
import time
from typing import List
from aiosv2.AiosSocket import AiosSocket
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.ConnectedMotor import CVPConverter
from aiosv2.CVP import CVP


# experimentally, a sampling time of 300Hz yields consistent results
SAMPLING_FREQUENCY = 300
SAMPLING_PERIOD = 1 / SAMPLING_FREQUENCY

class CVPStream:
    def __init__(self, socket: AiosSocket, motors: List[SafeMotor]):
        self.socket = socket
        self.motors = motors
        self.stop = False
        self.error = None
        self.readThread = None
        self.requestThread = None
        self.converter = CVPConverter()

    def enable(self):
        self.readThread = threading.Thread(target=self.read)
        self.readThread.start()

        # self.requestThread = threading.Thread(target=self.request)
        # self.requestThread.start()

    def disable(self):
        self.stop = True
        self.readThread.join()
        # self.requestThread.join()
        
    def errored(self):
        return self.error

    def read(self):
        while True:
            if self.stop:
                break

            result = self.check_for_cvp()
            if result is None:
                continue

            try: 
                json_obj, ip = result

                for motor in self.motors:
                    if motor.getIP() == ip:
                        cvp = self.converter.parseCVP(json_obj)
                        motor.setCVP(cvp)
            except Exception as err:
                self.stop = True
                self.error = err

    def check_for_cvp(self):
        try:
            json_obj, ip = self.socket.readJSON()
            if json_obj.get('reqTarget') != '/m1/CVP':
                return None
            return json_obj, ip
        except:
            return None
        
        

    def request(self):
        while True:
            if self.stop:
                break
            
            try:
                for motor in self.motors:
                    motor.requestCVP()
                time.sleep(SAMPLING_PERIOD)
            except: 
                pass
            