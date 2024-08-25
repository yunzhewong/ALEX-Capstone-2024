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
    
    plot(corrected_times, position_degrees, "DisplayName", string(current_commands(i)))
end
legend

%K_t*i = mgl*sin(theta) + F_kinetic
% sin(theta) = (K_t/mgl)*i - F_kinetic/(m*g*l)
y = sind(final_positions(cutoff_index:end));
x = average_currents(cutoff_index:end);
p = polyfit(x, y, 1);

Kt = 0.124 * 120;
m = 5.77;
g = 9.81;
l = Kt / (m*g*p(1));
F_kinetic = -p(2) * m * g *l;
STATIC_CURRENT = 0.7;
F_static = STATIC_CURRENT * Kt;

fprintf("Effective Length: %.4f\n", l)
fprintf("Effective Kinetic Friction: %.4f\n", F_kinetic)
fprintf("Effective Static Friction: %.4f\n", F_static)



spaced_currents = 0:0.01:3;
input_torques = Kt * spaced_currents;

expected_sin_positions = zeros(1, numel(spaced_currents));
for i = 1:numel(spaced_currents)
    if input_torques(i) >= F_static
        expected_sin_positions(i) = (input_torques(i) - F_kinetic) / (m*g*l);
    end
end



figure
hold on
plot(average_currents, sind(final_positions))
plot(spaced_currents, expected_sin_positions);
title("Current vs Sine of Final Calf Position")
legend("Measured Results", "Modelled Results", "Location", "southeast")
xlabel("Mean Current")
ylabel("sin(\theta)")

figure
hold on
plot(average_currents, final_positions);
plot(spaced_currents, asind(expected_sin_positions))
title("Current vs Angle Final Calf Position")
legend("Measured Results", "Modelled Results", "Location", "southeast")
xlabel("Mean Current")
ylabel("Angle (degrees)")

