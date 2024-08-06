close all
velocity_commands = 0:0.1:1.7;
resulting_velocities = zeros(1, numel(velocity_commands));

figure
hold on
for i = 1:numel(velocity_commands)
    data = readmatrix("./data/exo vel/step" + sprintf('%0.2f',velocity_commands(i)) + "rads.csv");
    
    times = data(:, 1);
    corrected_times = times - times(1);
    velocities = data(:, 3);
    resulting_velocities(i) = mean(velocities(floor(end / 10), end));

    plot(corrected_times, velocities);
end

figure
plot(velocity_commands, resulting_velocities)

bestfit = polyfit(velocity_commands, resulting_velocities, 1);
disp(bestfit)

% there ratio is not completely 1-1
% y = 0.9678x - 0.0229
