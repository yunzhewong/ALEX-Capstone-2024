close all
[times, currents, velocities, positions] = read_data("./knee.csv");
[times_2, currents_2, velocities_2, positions_2] = read_data("./kneereal.csv");

reference = readmatrix('kneetraj.csv');
ref_times = reference(:, 1);
ref_velocities = reference(:, 2);
ref_pos = reference(:, 3);

% knee
t2 = 10.363;
t1 = 0.997;

diff = t2 - t1;
adjust_times = times - diff;

fig = figure;
fig.Position = [50, 50, 900, 400];

plot(adjust_times, currents)
hold on
plot(times_2, currents_2)
xlim([0, 40])
xlabel("Time (s)")
ylabel("Current (A)")
legend("Simulated", "Measured", "Location", "southeast")
title("Comparison of Predicted Current vs Measured Current")