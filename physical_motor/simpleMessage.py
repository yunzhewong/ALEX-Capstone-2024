import math
import time
import aios
import aiosv2
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    socket.sendJSON("10.10.10.17", 8080, {"message": "Hello World"})
