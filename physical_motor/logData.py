import math
import time
import aios
import aiosv2
import numpy as np
import matplotlib.pyplot as plt

FREQUENCY = 0.01
SAVE_NAME = "lower_motor_pi_8.csv"

## For sinusoial waveform
AMPLITUDE = 2  # Amplitude of the sinusoidal waveform (in amps)
OFFSET = 0
FREQUENCY_SIGNAL = 0.5  # Frequency of the sinusoidal waveform (in Hz)


# Store currents, velocities, positions and a count of logged data points
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

        ax3.plot(x, np.array(self.positions), label="Position (rad)", color="green")
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

        data = np.column_stack(
            (
                time,
                currents,
                velocities,
                positions,
            )
        )

        header = "Time, Currents, Velocities, Positions"

        # Save data to CSV file
        np.savetxt(
            name,
            data,
            delimiter=",",
            header=header,
            comments="",
        )


# in this code, the motor actuates to pi / 8, 0, -pi/8, and back to 0
# prints the logged data across current, velocity, and position
if __name__ == "__main__":
    socket = aiosv2.AiosSocket()
    twinMotor = aiosv2.TwinMotor(socket)
    twinMotor.enable()

    time.sleep(1)

    connection = twinMotor.bottomMotor

    WAITING_TIME_S = 5
    dataLog = DataLog()

    ## Generate sinusoidal current input
    t = np.arange(0, WAITING_TIME_S, FREQUENCY)
    sinusoidal_currents = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY_SIGNAL * t) + OFFSET

    # Loop to set sinusoidal current input
    for current in sinusoidal_currents:
        # Set current
        connection.setCurrent(current)

        # Log data
        dataLog.readConnection(connection, FREQUENCY)

    # ## Loop for free response input ##
    # while True:
    #     # Read current input from user or sensor
    #     current_input = float(input("Enter desired current input (amps): "))

    #     # Set current
    #     connection.setCurrent(current_input)

    #     # Log data
    #     dataLog.readConnection(connection, WAITING_TIME_S)

    #     # Ask if user wants to continue
    #     continue_response = input("Continue logging data? (y/n): ")
    #     if continue_response.lower() != 'y':
    #         break

    # aios.controlMode(aios.ControlMode.POSITION_CONTROL.value, connection.ip, 1)
    # connection.setPosition(0)
    # dataLog.readConnection(connection, WAITING_TIME_S)
    # connection.setPosition(math.pi / 8)
    # dataLog.readConnection(connection, WAITING_TIME_S)
    # connection.setPosition(0)
    # dataLog.readConnection(connection, WAITING_TIME_S)
    # connection.setPosition(-math.pi / 8)
    # dataLog.readConnection(connection, WAITING_TIME_S)
    # connection.setPosition(0)
    # dataLog.readConnection(connection, WAITING_TIME_S)

    twinMotor.disable()

    dataLog.plot()
    dataLog.download(SAVE_NAME)
