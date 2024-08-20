close all

velocity_commands = 0.00:0.1:1.80;

for i = 1:numel(velocity_commands)
    filename = "./data/step" + compose("%1.2f", velocity_commands(i)) + "rads.csv";

    data = readmatrix(filename);

    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);

    figure
    plot(corrected_times(1:200), velocities(1:200))
end
