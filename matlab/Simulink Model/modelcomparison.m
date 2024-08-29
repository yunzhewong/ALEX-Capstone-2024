close all

FILENAME = '../data/vel1.csv';
data = readmatrix(FILENAME);    
times = data(:, 1);

corrected_times = times - times(1);
currents = data(:, 2);
velocities = data(:, 3);

figure
plot(corrected_times, currents)
hold on
plot(out.input.Time, out.input.Data)
title("Current over Time")
xlabel("Time (s)")
ylabel("Current (A)")
legend("Measured", "Modelled", "Location", "northeast")


figure
plot(corrected_times, velocities)
hold on
plot(out.velocity.Time, out.velocity.Data)
title("Velocity over Time")
xlabel("Time (s)")
ylabel("Velocity (rads^-1)")
legend("Measured", "Modelled", "Location", "southwest")