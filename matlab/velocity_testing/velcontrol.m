step = "./noi/vel2.csv";

data = readmatrix(step);

times = data(:, 1);
velocities = data(:, 3);

figure
plot(times, velocities)
yline(2, "LineWidth", 3, "LineStyle", ":")
ylim([0 2.2])
xlim([0 10])

legend("Measured Velocity", "Velocity Reference", "Location", "southeast")

xlabel("Time (s)")
ylabel("Velocity (rads^{-1})")
title("Controlled Velocity vs Time")