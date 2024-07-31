import time
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from aiosv2 import aios



CONTROL_BOX = "10.10.10.12"
IP = "10.10.10.30"

if __name__ == "__main__":
    # aios.getAbsEncoder(IP)
    # aios.getEncoderInfo(IP)
    # aios.getRoot(IP)
    # aios.controlMode(3, IP, 1)
    # aios.inputMode(3, IP, 1)
    # time.sleep(3)
    # aios.encoderIsReady(IP, 1)
    aios.enable(IP, 1)
    aios.controlMode(3, IP, 1)
    aios.inputMode(3, IP, 1)
    aios.getRoot(IP)
    aios.setInputPosition_pt(IP, 0, 0, 1)
    time.sleep(3)
    aios.getAbsEncoder(IP)
    aios.getCVP_pt(IP)
    aios.setInputPosition_pt(IP, 7.5, 0, 1)
    time.sleep(3)
    aios.getAbsEncoder(IP)
    aios.getCVP_pt(IP)
    aios.disable(IP, 1)
