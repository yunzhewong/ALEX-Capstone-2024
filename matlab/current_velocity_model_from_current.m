close all
clear

ITERATIONS = 0.02;
LOW = 0.8;
HIGH = 2.2;
current_commands = LOW:ITERATIONS:HIGH;

count = numel(current_commands);
steady_state_currents = zeros(1, count);
steady_state_velocities = zeros(1, count);

J_bs = zeros(1, count);

figure
hold on
for i=1:count
    fullname = "./data/exo batch 2/step" + compose("%1.2f", current_commands(i)) + "A.csv";

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

    plot(currents)
end

figure
plot(steady_state_currents, steady_state_velocities)
hold on
p = polyfit(steady_state_currents, steady_state_velocities, 1);
plot(steady_state_currents, p(1) * steady_state_currents + p(2))

average_Kt_b = p(1);

%v_ss_2 - v_ss_1 = Kt/b * (i_ss_2 - i_ss_1)
Kt_bs = zeros(1, count - 1);
for i = 1:count - 1
    Kt_bs(i) = (steady_state_velocities(i + 1) - steady_state_velocities(i)) / (steady_state_currents(i + 1) - steady_state_currents(i));
end

figure
plot(Kt_bs)
hold on
yline(average_Kt_b)



Fc_bs = zeros(1, count - 1);
for i = 1:count - 1
    Fc_bs(i) = average_Kt_b * steady_state_currents(i) - steady_state_velocities(i);
end

figure
plot(Fc_bs)

figure
plot(steady_state_currents)

figure
plot(steady_state_velocities)

consistent_Fc_b = mean(Fc_bs);
consistent_J_b = mean(J_bs);

figure
plot(steady_state_currents, steady_state_velocities)
hold on
plot(steady_state_currents, average_Kt_b * steady_state_currents - consistent_Fc_b)

%assumption
Kt = 0.124 * 120;
min_amperage = 0.80;

b = Kt / average_Kt_b
F_c = consistent_Fc_b * b
F_stat = Kt * min_amperage
J = consistent_J_b * b

function [match, index] = closest(values, searchValue) 
    [match, index] = min(abs(values - searchValue));
end

