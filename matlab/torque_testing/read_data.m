function [times, currents, velocities, positions] = read_data(filename)
    data = readmatrix(filename);

    uncorrected_times = data(:, 1);
    times = uncorrected_times(:, 1) - uncorrected_times(1);
    currents = data(:, 2);
    velocities = data(:, 3);
    positions = data(:, 4);
end