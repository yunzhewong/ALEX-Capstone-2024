close all
clear

current_commands = 0.00:0.05:3.05;
% current_commands = 1.00:0.20:2.2;
count = numel(current_commands);
steady_state_currents = zeros(1, count);
steady_state_velocities = zeros(1, count);

first_accels = zeros(1, count);
J_bs = zeros(1, count);

for i=1:count
    % fullname = "./data/exo1/step" + compose("%1.2f", current_commands(i)) + "A.csv";
    fullname = "./data/current" + compose("%1.2f", current_commands(i)) + "A.csv";

    data = readmatrix(fullname);
    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);

    steady_state_index = floor(numel(times) / 2);

    steady_state_currents(i) = mean(currents(steady_state_index:end));
    steady_state_velocities(i) = mean(velocities(steady_state_index:end));

    finalTime = corrected_times(numel(corrected_times));
   
    outputAfterOneTau = (1 - exp(-1)) * steady_state_velocities(i);

    [~, tauIndex] = closest(velocities, outputAfterOneTau);

    J_bs(i) = corrected_times(tauIndex);


    FIRST_COUNT = 20;
    first_velocities = velocities(1:FIRST_COUNT, 1);
    first_times = corrected_times(1:FIRST_COUNT, 1);
    accels = zeros(1, FIRST_COUNT);
    for j = 1:(FIRST_COUNT-1)
        v_diff = first_velocities(j + 1) - first_velocities(j);
        t_diff = first_times(j + 1) - first_times(j);
        accels(j) = v_diff/t_diff;
    end
    first_accels(i) = mean(accels);
end

figure
plot(steady_state_velocities)

MOVING_INDEX = 20;
SATURATION_INDEX = 50;
 
%assumption
Kt = 0.124 * 120;
F_kinetic = 8.1003;
F_static = 10.4160;

moving_currents = steady_state_currents(MOVING_INDEX:SATURATION_INDEX);
moving_velocities = steady_state_velocities(MOVING_INDEX:SATURATION_INDEX);
moving_accelerations = first_accels(MOVING_INDEX:SATURATION_INDEX);

expected_torques = Kt * moving_currents - F_kinetic;

% J_t*a_i = tau_i
J_t_measurements = expected_torques ./ moving_accelerations;

figure
plot(J_t_measurements)

J_t = mean(J_t_measurements);
fprintf("J_t = %.4f\n", J_t);

b_predictions = expected_torques ./ moving_velocities;

figure
plot(moving_currents, b_predictions)
title("Observed Current vs Predicted Damping Coefficient")
ylabel("Damping Coefficient")
xlabel("Current (A)")

%looks like b is a function of i, such that b(i) = b_2*i + b_1
pb = polyfit(moving_currents, b_predictions, 1);
b_2 = pb(1);
b_1 = pb(2);
fprintf("b(i) = %.4f*i + %.4f\n", b_2, b_1);

sim_currents = 0.0:0.01:2.2;
sim_b = b_2 * sim_currents + b_1;

figure
plot(moving_currents, b_predictions)
hold on
plot(sim_currents, sim_b)   
title("Current vs Predicted Damping Coefficient")
ylabel("Damping Coefficient")
xlabel("Current (A)")
legend("Observed Results", "Line of Best Fit")

expected_velocities = zeros(1, numel(sim_currents));
for i = 1:numel(sim_currents)
    motor_torque = Kt*sim_currents(i);
    if motor_torque > F_static
        expected_velocities(i) = (motor_torque - F_kinetic)/sim_b(i);
    end
end

figure
plot(sim_currents, expected_velocities)
hold on
plot(steady_state_currents, steady_state_velocities)
title("Current vs Velocity")
ylabel("Velocity (rad/s)")
xlabel("Current (A)")
legend("Observed Results", "Modelled Results")

function [match, index] = closest(values, searchValue) 
    [match, index] = min(abs(values - searchValue));
end

