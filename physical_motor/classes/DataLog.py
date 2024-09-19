import matplotlib.pyplot as plt
import numpy as np
from aiosv2.CVP import CVP
import pandas as pd
import os

class DataLog:
    def __init__(self, desired_trajectory_bottom=None, desired_trajectory_top=None, desired_velocity_bottom=None, desired_velocity_top=None):
        self.currents_top = []
        self.velocities_top = []
        self.positions_top = []
        self.currents_bottom = []
        self.velocities_bottom = []
        self.positions_bottom = []
        self.times = []

        self.desired_trajectory_bottom = desired_trajectory_bottom
        self.desired_trajectory_top = desired_trajectory_top
        self.desired_velocity_bottom = desired_velocity_bottom
        self.desired_velocity_top = desired_velocity_top

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
        
        # Plot the desired velocities if provided
        if self.desired_velocity_bottom is not None:
            desired_vel_bottom = [np.polyval(self.desired_velocity_bottom[::-1], t) for t in x]
            ax2.plot(x, desired_vel_bottom, label="Desired Velocity Bottom", linestyle='--')
        if self.desired_velocity_top is not None:
            desired_vel_top = [np.polyval(self.desired_velocity_top[::-1], t) for t in x]
            ax2.plot(x, desired_vel_top, label="Desired Velocity Top", linestyle='--', color="green")

        ax2.set_ylabel("Velocity (rad/s)")
        ax2.legend()

        ax3.plot(x, np.array(self.positions_top), label="Top Motor Position (rad)", color="green")
        ax3.plot(x, np.array(self.positions_bottom), label="Bottom Motor Position (rad)", color="magenta")
        # Plot the desired trajectories if provided
        
        if self.desired_trajectory_bottom is not None:
            desired_traj_bottom = [np.polyval(self.desired_trajectory_bottom[::-1], t) for t in x]
            ax3.plot(x, desired_traj_bottom, label="Desired Trajectory Bottom", linestyle='--')
        if self.desired_trajectory_top is not None:
            desired_traj_top = [np.polyval(self.desired_trajectory_top[::-1], t) for t in x]
            ax3.plot(x, desired_traj_top, label="Desired Trajectory Top", linestyle='--')

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

class CVPPlot():
    def __init__(self):
        self.times = []
        self.currents = []
        self.velocities = []
        self.positions = []

        # self.desired_trajectory = desired_trajectory
        # self.desired_velocity = desired_velocity
    
    def addCVP(self, current_time: float, cvp: CVP):
        self.currents.append(cvp.current)
        self.velocities.append(cvp.velocity)
        self.positions.append(cvp.position)
        self.times.append(current_time)

    
    def plot(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        x = np.array(self.times)

        ax1.plot(x, np.array(self.currents), color="blue")
        ax1.set_ylabel("Current (A)")

        ax2.plot(x, np.array(self.velocities), color="red")
        ax2.set_ylabel("Velocity (rad/s)")

        # Plot the desired velocities if provided
        # if self.desired_velocity is not None:
        #     desired_vel = [np.polyval(self.desired_velocity[::-1], t) for t in x]
        #     ax2.plot(x, desired_vel, label="Desired Velocity", linestyle='--')

        ax3.plot(x, np.array(self.positions), color="green")
        ax3.set_ylabel("Position (rad)")

        # Plot the desired trajectories if provided
        # if self.desired_trajectory_bottom is not None:
        #     desired_traj = [np.polyval(self.desired_trajectory[::-1], t) for t in x]
        #     ax3.plot(x, desired_traj, label="Desired Trajectory", linestyle='--')
        
        fig.suptitle("Motor Values")

        plt.tight_layout()

        plt.show()

    def download(self, name):
        data = {
            'Time': self.times,
            'Current': self.currents,
            'Velocity': self.velocities,
            'Position': self.positions
        }

        df = pd.DataFrame(data)
        df.to_csv('name', index=False)
        
