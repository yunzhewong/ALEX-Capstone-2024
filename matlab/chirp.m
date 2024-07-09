close all

% [times, currents, velocities, positions] = load_data('chirp025.csv');


data = readmatrix("data.csv");

times = data(:, 1);
currents = data(:, 5);
velocities = data(:, 6);


start = 1460;
last = 52600;
slicedTimes = times(start:last) - times(start);
slicedCurrents = currents(start:last);
slicedVelocities= velocities(start:last);

plot(1:numel(currents), currents)
% chirp_identification(times, currents', velocities')


