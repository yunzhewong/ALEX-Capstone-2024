close all

current_data = readmatrix("current");
times = current_data(:, 1);
current_times = times(:, 1) - times(1);
current_currents = current_data(:, 2);
current_velocities = current_data(:, 3);

velocity_data = readmatrix("velocity");
times = velocity_data(:, 1);
velocity_times = times(:, 1) - times(1);
velocity_currents = velocity_data(:, 2);
velocity_velocities = velocity_data(:, 3);

figure
hold on
plot(velocity_times, velocity_currents)
plot(current_times, current_currents)
title("Current Comparison")
xlabel("Time (s)")
ylabel("Current (A)")
legend("Constant Velocity Error Signal", "Equivalent Current Command", "Location", "southeast")

figure
hold on
plot(velocity_times, velocity_velocities)
plot(current_times, current_velocities)
title("Velocity Comparison")
xlabel("Time (s)")
ylabel("Velocity (rad/s)")
legend("Constant Velocity Error Signal", "Equivalent Current Command", "Location", "southeast")