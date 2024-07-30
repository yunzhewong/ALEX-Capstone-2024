from typing import List
from aiosv2.SafeMotorOperation import SafeMotor


class CSVWriter():
    def __init__(self, name: str, motors: List[SafeMotor]):
        self.f = open(name, 'w')
        self.f.write(f"Time,")
        for motor in motors:
            ip = motor.getIP()
            self.f.write(f"{ip} Current, {ip} Velocity, {ip} Position")
        self.f.write('\n')

    def addCVP(self, time: float, motors: List[SafeMotor]):
        self.f.write(f"{time},")
        for motor in motors:
            cvp = motor.getCVP()
            if cvp is None:
                self.f.write(f"0, 0, 0")
                return
            self.f.write(f"{cvp.current},{cvp.velocity},{cvp.position}")
                
        self.f.write(f"\n")
    def close(self):
        self.f.close()