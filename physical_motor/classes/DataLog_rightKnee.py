import matplotlib.pyplot as plt
import numpy as np

class DataLog_rightKnee:
    def __init__(self):
        self.currents = []
        self.velocities = []
        self.positions = []
        self.times = []

    def addCVP(self, current_time, cvp):
        self.currents.append(cvp.current)
        self.velocities.append(cvp.velocity)
        self.positions.append(cvp.position)
        self.times.append(current_time)

    def plot(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        x = np.array(self.times)

        ax1.plot(x, np.array(self.currents_top), label="Top Motor Current", color="blue")
        ax1.set_ylabel("Current (A)")
        ax1.legend()

        ax2.plot(x, np.array(self.velocities_top), label="Top Motor Velocity", color="red")
        ax2.set_ylabel("Velocity (rad/s)")
        ax2.legend()

        ax3.plot(x, np.array(self.positions_top), label="Top Motor Position (rad)", color="green")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Position (rad)")
        ax3.legend()

        # Add a title to the entire figure
        fig.suptitle("Motor Values")

        # Adjust layout to prevent overlapping
        plt.tight_layout()

        # Show the plot
        plt.show()

    def download(self, name):
        times = np.array(self.times)
        currents_top = np.array(self.currents)
        velocities_top = np.array(self.velocities)
        positions_top = np.array(self.positions)

        data = np.column_stack(
            (
                times,
                currents_top,
                velocities_top,
                positions_top,
            )
        )

        header = "Time, Motor Current, Motor Velocities, Motor Positions"

        # Save data to CSV file
        np.savetxt(
            name,
            data,
            delimiter=",",
            header=header,
            comments="",
        )
