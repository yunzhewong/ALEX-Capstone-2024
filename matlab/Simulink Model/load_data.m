function [times, currents, velocities, positions] = load_data(filename)
    data = readmatrix(filename);
    
    times = data(:, 1);
    currents = data(:, 2);
    velocities = data(:, 3);
    positions = data(:, 4);
end