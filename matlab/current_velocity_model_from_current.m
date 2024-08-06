close all

iters = 0.02;
low = 0.6;
high = 2.16;
amperage = low:iters:high;


count = numel(amperage);
K_bs = zeros(1, count);
J_bs = zeros(1, count);

for i=1:count
    current = amperage(i);
    fullname = "./data/exo batch 2/step" + compose("%1.2f", current) + "A.csv";

    data = readmatrix(fullname);

    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);

    averageCurrent = mean(currents);

    finalTime = corrected_times(numel(corrected_times));
    
    steadyStateTime = finalTime / 2;

    [~, steadyStateIndex] = closest(corrected_times, steadyStateTime);

    steadyState = velocities(steadyStateIndex:numel(velocities));

    expectedSteadyState = mean(steadyState);
    % = K / b, K = K_ti - F
    K_bs(i) = expectedSteadyState;

    outputAfterOneTau = (1 - exp(-1)) * expectedSteadyState;

    [~, tauIndex] = closest(velocities, outputAfterOneTau);

    time = corrected_times(tauIndex);
    % = J / b

    J_bs(i) = time;
    
    % figure
    % plot(corrected_times, velocities)
    % hold on
    % xline(time)
    % yline(expectedSteadyState)
end

Kt_bs = zeros(1, count - 1);
for i = 1:count - 1
    Kt_bs(i) = (K_bs(i + 1) - K_bs(i)) / iters;
end

average_Kt_b = mean(Kt_bs);
Fc_bs = K_bs - average_Kt_b * amperage;

average_Fc_b = mean(Fc_bs);
average_J_b = mean(J_bs);

fprintf("Average Kt/b: %f\n", average_Kt_b)
fprintf("Average Fc/b: %f\n", average_Fc_b)
fprintf("Average J/b: %f\n", average_J_b)

Kt_bs
Fc_bs
J_bs

figure
plot(amperage, K_bs)
hold on

p = polyfit(amperage, K_bs, 2)

quadratic_fit = p(1) * (amperage .* amperage) + p(2) * amperage + p(3)
plot(amperage, quadratic_fit)

figure
plot(amperage(1:numel(amperage) - 1), Kt_bs)

figure
plot(amperage, Fc_bs)

figure
plot(amperage, J_bs)

%assumption
K_t = 3.563;
min_amperage = 0.48;

b = K_t / average_Kt_b
F_c = average_Fc_b * b
F_stat = K_t * min_amperage
J = average_J_b * b
function [match, index] = closest(values, searchValue) 
    [match, index] = min(abs(values - searchValue));
end

