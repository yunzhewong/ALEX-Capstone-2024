import time
import aiosv2
import matplotlib.pyplot as plt
import numpy as np

MINI_PAUSE = 0.001


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
