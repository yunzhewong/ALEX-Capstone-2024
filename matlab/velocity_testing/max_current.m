close all

velocity_commands = 0.00:0.05:1.95;

max_currents = zeros(1, numel(velocity_commands));
average_velocities = zeros(1, numel(velocity_commands));

figure
hold on
for i = 1:numel(velocity_commands)
    filename = "./test3/vel" + compose("%1.2f", velocity_commands(i)) + "rads.csv";

    data = readmatrix(filename);

    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);

    max_currents(i) = max(currents);
    average_velocities(i) = mean(velocities);

    plot(corrected_times, velocities)
end

figure
plot(velocity_commands, max_currents)
