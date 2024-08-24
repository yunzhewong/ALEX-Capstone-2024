close all

current_commands = 0.00:0.05:1.60;
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
    filename = "./test2/current" + compose("%1.2f", current_commands(i)) + "A.csv";

    data = readmatrix(filename);

    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);
    positions = data(:, 4);

    position_degrees = positions * 180 / pi;

    average_currents(i) = mean(currents);
    final_positions(i) = position_degrees(end);
    
    plot(corrected_times, position_degrees)
end

figure
plot(current_commands, average_currents)

%K_t*i = mgl*sin(theta) + F_c
Kt = 0.124 * 120;
m = 5.77;
g = 9.81;

% sin(theta) = (K_t/mgl)*i - F_c/(m*g*l)

p = polyfit(average_currents(cutoff_index:end), sind(final_positions(cutoff_index:end)), 1);
l = Kt / (m*g*p(1));
F_c = -p(2) * m * g *l;

fprintf("Effective Length: %.4f\n", l)
fprintf("Effective Static Friction: %.4f\n", F_c)


spaced_currents = 0:0.05:3;
expected_sin_positions = (Kt * spaced_currents - F_c) / (m*g*l);

figure
hold on
plot(average_currents, sind(final_positions))
plot(spaced_currents, expected_sin_positions);
title("Current vs Sine of Final Calf Position")
legend("Measured Results", "Line of Best Fit", "Location", "southeast")
xlabel("Mean Current")
ylabel("sin(\theta)")

figure
hold on
plot(average_currents, final_positions);
plot(spaced_currents, asind(expected_sin_positions))
title("Current vs Angle Final Calf Position")
legend("Measured Results", "Model", "Location", "southeast")
xlabel("Mean Current")
ylabel("Angle (degrees)")

