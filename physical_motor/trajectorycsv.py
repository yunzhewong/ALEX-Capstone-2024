from rebuild import EXO24Trajectory

DURATION = 86


if __name__ == "__main__":
    trajectory = EXO24Trajectory.Exo24Trajectory()
    file = open("kneetraj.csv", "w")
    file.write("Time, Velocity, Position\n")

    time = 0

    while time <= DURATION:
        reference_position, reference_velocity = trajectory.get_state(time)
        file.write(f"{time}, {reference_velocity[5]}, {reference_position[5]}\n")
        time += 0.001

    file.close()
