
import datetime
from typing import List
from aiosv2.SafeMotorOperation import SafeMotor
from aiosv2.readConfig import writeConfigurationJSON


EPSILON = 0.01
CALIBRATION_SPEED = 0.3
ZERO_VELOCITY_COUNT = 10

class MotorCalibration():
    def __init__(self, config: dict, position: float | None):
        self.direction = config['direction']
        self.limit = config['limit']
        self.readLimitPosition = position
        self.zeroVelocityCount = 0
        self.centered = False
        if position is not None:
            self.centered = True


class CalibrationState():
    def __init__(self, setupConfig: dict, savePath: List[str], positions: List[float | None]):
        self.motors: List[SafeMotor] | None = None
        self.calibrations: List[MotorCalibration] = []

        for i, config in enumerate(setupConfig["motors"]):
            self.calibrations.append(MotorCalibration(config, positions[i]))

        self.savePath = savePath

    def allMotorsCentered(self):
        count = 0 
        for calibration in self.calibrations:
            if calibration.centered:
                count += 1
        
        return count == len(self.calibrations)
    
    def iterate(self):
        for i, motor in enumerate(self.motors):
            cvp = motor.getNonCalibratedCVP()

            if abs(cvp.velocity) < EPSILON:
                self.calibrations[i].zeroVelocityCount += 1

            atBoundary = self.calibrations[i].zeroVelocityCount > ZERO_VELOCITY_COUNT
            positionNotFoundYet = self.calibrations[i].readLimitPosition is None  

            if atBoundary and positionNotFoundYet:
                print(f"{motor.getIP()} Found Limit")
                self.calibrations[i].readLimitPosition = cvp.position

            calibrationVelocity = self.calibrations[i].direction * CALIBRATION_SPEED

            if self.calibrations[i].readLimitPosition is None:
                motor.setVelocity(calibrationVelocity)
            else:
                position_adjustment = self.calibrations[i].limit - self.calibrations[i].readLimitPosition
                truePosition = cvp.position + position_adjustment
                
                if self.calibrations[i].centered or abs(truePosition) < EPSILON:
                    motor.setVelocity(0)
                    self.calibrations[i].centered = True
                    if self.allMotorsCentered():
                        writeConfigurationJSON({
                            "date": datetime.datetime.now().isoformat(),
                            "adjustments": [self.calibrations[i].limit - self.calibrations[i].readLimitPosition for i in range(len(self.calibrations))]
                        }, self.savePath)
                        raise Exception("All Motors Calibrated")
                else:
                    motor.setVelocity(-1 * calibrationVelocity)

                    
