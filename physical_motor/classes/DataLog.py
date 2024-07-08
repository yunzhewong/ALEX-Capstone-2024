import matplotlib.pyplot as plt
import numpy as np

class DataLog:
    def __init__(self):
        self.currents_top = []
        self.velocities_top = []
        self.positions_top = []
        self.currents_bottom = []
        self.velocities_bottom = []
        self.positions_bottom = []
        self.times = []

    def addCVP(self, current_time, cvp_top, cvp_bottom):
        self.currents_top.append(cvp_top.current)
        self.velocities_top.append(cvp_top.velocity)
        self.positions_top.append(cvp_top.position)
        self.currents_bottom.append(cvp_bottom.current)
        self.velocities_bottom.append(cvp_bottom.velocity)
        self.positions_bottom.append(cvp_bottom.position)
        self.times.append(current_time)

    def plot(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        x = np.array(self.times)

        ax1.plot(x, np.array(self.currents_top), label="Top Motor Current", color="blue")
        ax1.plot(x, np.array(self.currents_bottom), label="Bottom Motor Current", color="purple")
        ax1.set_ylabel("Current (A)")
        ax1.legend()

        ax2.plot(x, np.array(self.velocities_top), label="Top Motor Velocity", color="red")
        ax2.plot(x, np.array(self.velocities_bottom), label="Bottom Motor Velocity", color="orange")
        ax2.set_ylabel("Velocity (rad/s)")
        ax2.legend()

        ax3.plot(x, np.array(self.positions_top), label="Top Motor Position (rad)", color="green")
        ax3.plot(x, np.array(self.positions_bottom), label="Bottom Motor Position (rad)", color="magenta")
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
        currents_top = np.array(self.currents_top)
        velocities_top = np.array(self.velocities_top)
        positions_top = np.array(self.positions_top)
        currents_bottom = np.array(self.currents_bottom)
        velocities_bottom = np.array(self.velocities_bottom)
        positions_bottom = np.array(self.positions_bottom)

        data = np.column_stack(
            (
                times,
                currents_top,
                velocities_top,
                positions_top,
                currents_bottom,
                velocities_bottom,
                positions_bottom,
            )
        )

        header = "Time, Top Motor Current, Top Motor Velocities, Top Motor Positions, Bottom Motor Current, Bottom Motor Velocities, Bottom Motor Positions"

        # Save data to CSV file
        np.savetxt(
            name,
            data,
            delimiter=",",
            header=header,
            comments="",
        )
