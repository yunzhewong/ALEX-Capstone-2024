import aios
import time
import threading
import numpy as np

Server_IP_list = []
IP = "10.10.10.17"


def main():
    aios.enable(IP, 1)
    aios.getState(IP, 1)
    aios.encoderIsReady(IP, 1)
    aios.getError(IP, 1)
    aios.clearError(IP, 1)
    aios.reboot(IP)




if __name__ == '__main__':
    main()
