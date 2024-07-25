from aiosv2 import AiosSocket

if __name__ == "__main__":
    socket = AiosSocket()
    foundIps = socket.readConnectedAddresses()
    print(foundIps)
