close all

current_commands = 0.8:0.1:1.80;

figure
hold on

average_currents = zeros(1, numel(current_commands));
final_positions = zeros(1, numel(current_commands));

for i = 1:numel(current_commands)
    filename = "./data/lowerleg" + compose("%1.2f", current_commands(i)) + ".csv";

    data = readmatrix(filename);

    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);
    positions = data(:, 4);

    position_degrees = positions * 180 / pi;

    average_currents(i) = mean(currents);
    final_positions(i) = position_degrees(end);
end

%K_t*i = mgl*sin(theta) + F_c
Kt = 0.124 * 120;
m = 5.77;
g = 9.81;

% sin(theta) = (K_t/mgl)*i - F_c/(m*g*l)

p = polyfit(average_currents, sind(final_positions), 1);
l = Kt / (m*g*p(1));

F_c = -p(2) * m * g *l;

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

