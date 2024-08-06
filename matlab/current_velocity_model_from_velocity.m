close all

ITERATION = 0.1;
LOW = 0.1;
HIGH = 1;
velocity_commands = LOW:ITERATION:HIGH;

count = numel(velocity_commands);
steady_state_currents = zeros(1, count);
steady_state_velocities = zeros(1, count);

figure
hold on
for i=1:count
    active_velocity = velocity_commands(i);
    fullname = "./data/exo vel/step" + compose("%1.2f", active_velocity) + "rads.csv";

    data = readmatrix(fullname);
    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);

    indexes = numel(velocities);
    steadyStateIndex = floor(indexes / 2);

    steady_state_currents(i) = mean(currents(steadyStateIndex:end));
    steady_state_velocities(i) = mean(velocities(steadyStateIndex:end));

    plot(currents)
end

% Kt*i_ss - bv_ss - F_c = 0
% v_ss = Kt/b * i_ss - F_c/b

figure
plot(steady_state_currents, steady_state_velocities)
hold on
plot(steady_state_currents, average_Kt_b * steady_state_currents - consistent_Fc_b)
