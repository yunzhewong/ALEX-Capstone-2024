import aiosv2

if __name__ == "__main__":
    socket = aiosv2.AiosSocket()

    while True:
        json, ip = socket.readJSON()
        print(json)
