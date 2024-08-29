close all

current_commands = 0.00:0.05:4.55;
CUTOFF_CURRENT = 0.75;
cutoff_index = (CUTOFF_CURRENT / 0.05) + 1;

average_currents = zeros(1, numel(current_commands));
final_positions = zeros(1, numel(current_commands));

figure
hold on
title("Position vs Time for Incrementing Currents")
xlabel("Test Time (s)")
ylabel("Position (degrees)")

for i = 1:numel(current_commands)
    filename = "./rightextensor/current" + compose("%1.2f", current_commands(i)) + "A.csv";

    data = readmatrix(filename);

    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);
    positions = data(:, 4);

    position_degrees = positions * 180 / pi;

    average_currents(i) = mean(currents);
    final_positions(i) = position_degrees(end);
    
    plot(corrected_times, position_degrees, "DisplayName", string(current_commands(i)))
end
legend

%K_t*i = mgl*sin(theta) + F_kinetic
% sin(theta) = (K_t/mgl)*i - F_kinetic/(m*g*l)

fit_torques(-1 * average_currents, -1 * final_positions, 14)

