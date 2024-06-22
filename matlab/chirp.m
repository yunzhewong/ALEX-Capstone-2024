close all

filename = 'chirp025.csv';
data = readmatrix(filename);

times = data(:, 1);
currents = data(:, 2);
velocities = data(:, 3);
positions = data(:, 4);

chirp_identification(times, currents', velocities')


