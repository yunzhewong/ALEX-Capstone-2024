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

    position_degrees = -1 * positions * 180 / pi;

    average_currents(i) = -1 * mean(currents);
    final_positions(i) = position_degrees(end);
    
    plot(corrected_times, position_degrees, "DisplayName", string(current_commands(i)))
end

dcm = datacursormode(gcf);
set(dcm, 'UpdateFcn', @myUpdateFcn);
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



spaced_currents = 0:0.02:5;
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
title("Current vs Sine of Final Right Extensor Position")
legend("Measured Results", "Modelled Results", "Location", "southeast")
xlabel("Mean Current")
ylabel("sin(\theta)")

figure
hold on
plot(average_currents, final_positions);
plot(spaced_currents, asind(expected_sin_positions))
title("Current vs Final Right Extensor Position Angle")
legend("Measured Results", "Modelled Results", "Location", "southeast")
xlabel("Mean Current")
ylabel("Angle (degrees)")


% Custom update function to display plot name
function output_txt = myUpdateFcn(~, event_obj)
    % Get the target (the plot line)
    target = get(event_obj, 'Target');
    
    % Get the display name
    displayName = get(target, 'DisplayName');
    
    % Get the position of the data cursor
    pos = get(event_obj, 'Position');
    
    % Construct the output text
    output_txt = {['X: ', num2str(pos(1))], ...
                  ['Y: ', num2str(pos(2))], ...
                  ['Name: ', displayName]};
end

