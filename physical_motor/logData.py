import math
import time
import aiosv2
import numpy as np
import matplotlib.pyplot as plt

SAMPLE_PERIOD = 0.01
# SAVE_NAME = "lower_motor_pi_8.csv"
SAVE_NAME = "jerk_motor.csv"

## For jerk trajectory (6-degree)
POLYNOMIAL_COEFFICIENTS = [1e-6, -1e-5, 1e-4, -1e-3, 1e-2, -1e-1, 1]

# ## For sinusoidal waveform
# AMPLITUDE = 1.5  # Amplitude of the sinusoidal waveform (in amps)
# OFFSET = 0
# FREQUENCY_SIGNAL = 0.5  # Frequency of the sinusoidal waveform (in Hz)


# Store currents, velocities, positions, and a count of logged data points
class DataLog:
    def __init__(self):
        self.currents = []
        self.velocities = []
        self.positions = []
        self.times = []

    def addCVP(self, current_time, cvp: aiosv2.CVP):
        self.currents.append(cvp.current)
        self.velocities.append(cvp.velocity)
        self.positions.append(cvp.position)
        self.times.append(current_time)

    def readConnection(self, connection: aiosv2.ConnectedMotor, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            cvp = connection.getCVP()
            current_time = time.time() - start_time
            self.addCVP(current_time, cvp)
            time.sleep(SAMPLE_PERIOD)

    def plot(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
        x = np.array(self.times)

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
        times = np.array(self.times)
        currents = np.array(self.currents)
        velocities = np.array(self.velocities)
        positions = np.array(self.positions)

        data = np.column_stack(
            (
                times,
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

    WAITING_TIME_S = 15
    dataLog = DataLog()

    ## For polynomial current ##
    # Generate polynomial current input for three phases
    t = np.arange(0, 2, SAMPLE_PERIOD)
    phase1 = np.polyval(POLYNOMIAL_COEFFICIENTS, t)  # Move in one direction
    phase2 = -np.polyval(POLYNOMIAL_COEFFICIENTS, t)  # Move in the opposite direction

    # Define number of cycles
    num_cycles = 5

    # Repeat phases to create the desired number of cycles
    polynomial_currents = np.tile(np.concatenate((phase1, phase2)), num_cycles)

    # Loop to set polynomial current input
    start_time = time.time()
    for current in polynomial_currents:
        # Set current
        connection.setCurrent(current)

        # Log data
        cvp = connection.getCVP()
        current_time = time.time() - start_time
        dataLog.addCVP(current_time, cvp)

        time.sleep(SAMPLE_PERIOD)

    # ## Generate sinusoidal current input
    # t = np.arange(0, WAITING_TIME_S, SAMPLE_PERIOD)
    # sinusoidal_currents = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY_SIGNAL * t) + OFFSET

    # # Loop to set sinusoidal current input
    # start_time = time.time()
    # for i, current in enumerate(sinusoidal_currents):
    #     # Set current
    #     connection.setCurrent(current)

    #     # Log data
    #     cvp = connection.getCVP()
    #     current_time = time.time() - start_time
    #     dataLog.addCVP(current_time, cvp)

    #     time.sleep(SAMPLE_PERIOD)

    twinMotor.disable()

    dataLog.plot()
    dataLog.download(SAVE_NAME)
