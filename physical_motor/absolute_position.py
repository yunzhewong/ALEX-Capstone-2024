import time
from aiosv2.RightKneeExoMotor import setup_teardown_rightknee_exomotor, RightKneeExoMotor
from aiosv2 import aios



CONTROL_BOX = "10.10.10.12"
IP = "10.10.10.30"

if __name__ == "__main__":
    aios.getRoot(IP)
    aios.enable(IP, 1)
    aios.getCVP_pt(IP)
    time.sleep(3)    
    aios.setInputPosition_pt(IP, 7.5, 0, 1)
    time.sleep(3)
    aios.getCVP_pt(IP)
    aios.disable(IP, 1)
