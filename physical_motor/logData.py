import math
import time
import aios
import aiosv2
import numpy as np
import matplotlib.pyplot as plt

FREQUENCY = 0.01
SAVE_NAME = "outputtest.csv"


class DataLog:
    def __init__(self):
        self.currents = []
        self.velocities = []
        self.positions = []
        self.count = 0

    def addCVP(self, cvp: aiosv2.CVP):
        self.currents.append(cvp.current)
        self.velocities.append(cvp.velocity)
        self.positions.append(cvp.position)
        self.count += 1

    def readConnection(self, connection: aiosv2.ConnectedMotor, timeS):
        iterations = int(timeS / FREQUENCY)

        for _ in range(iterations):
            self.addCVP(connection.getCVP())
            time.sleep(FREQUENCY)

    def plot(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
        x = np.linspace(0, self.count * FREQUENCY, self.count)

        ax1.plot(x, np.array(self.currents), label="Current", color="blue")
        ax1.set_ylabel("Current")
        ax1.legend()

        ax2.plot(x, np.array(self.velocities), label="Velocity", color="red")
        ax2.set_ylabel("Velocity")
        ax2.legend()

        ax3.plot(x, np.array(self.positions), label="Position", color="green")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Position")
        ax3.legend()

        # Add a title to the entire figure
        fig.suptitle("Motor Values")

        # Adjust layout to prevent overlapping
        plt.tight_layout()

        # Show the plot
        plt.show()

    def download(self, name: str):
        time = np.linspace(0, self.count * FREQUENCY, self.count)
        currents = np.array(self.currents)
        velocities = np.array(self.velocities)
        positions = np.array(self.positions)

        # Stack arrays horizontally
        data = np.column_stack((time, currents, velocities, positions))

        # Save data to CSV file
        np.savetxt(
            name,
            data,
            delimiter=",",
            header="Time, Currents,Velocities,Positions",
            comments="",
        )


if __name__ == "__main__":
    connected_addresses = aiosv2.get_addresses()
    connected_addresses.enable()

    connectedMotors = connected_addresses.getConnectedMotors()
    time.sleep(1)
    print("MOVEMENT")

    connection = connectedMotors[1]

    WAITING_TIME_S = 0.4
    dataLog = DataLog()
    aios.controlMode(aios.ControlMode.POSITION_CONTROL.value, connection.ip, 1)
    connection.setPosition(0)
    dataLog.readConnection(connection, WAITING_TIME_S)
    connection.setPosition(math.pi / 8)
    dataLog.readConnection(connection, WAITING_TIME_S)
    connection.setPosition(0)
    dataLog.readConnection(connection, WAITING_TIME_S)
    connection.setPosition(-math.pi / 8)
    dataLog.readConnection(connection, WAITING_TIME_S)
    connection.setPosition(0)
    dataLog.readConnection(connection, WAITING_TIME_S)


connected_addresses.disable()

dataLog.plot()
dataLog.download()
