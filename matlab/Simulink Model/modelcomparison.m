close all

FILENAME = '../data/exo batch 2/step2.00A.csv';
data = readmatrix(FILENAME);    
times = data(:, 1);

corrected_times = times - times(1);
currents = data(:, 2);
velocities = data(:, 3);

figure
plot(corrected_times, currents)

figure
plot(corrected_times, velocities)
hold on
plot(out.velocity.Time, out.velocity.Data)