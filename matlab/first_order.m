close all

amperage = 0.46:0.02:1;
iters = 0.02;

count = numel(amperage);
K_bs = zeros(1, count);
J_bs = zeros(1, count);

for i=1:count
    current = amperage(i);
    fullname = "./data/step" + compose("%1.2f", current) + "A.csv";

    data = readmatrix(fullname);

    times = data(:, 1);
    corrected_times = times(:, 1) - times(1);
    currents = data(:, 2);
    velocities = data(:, 3);

    averageCurrent = mean(currents);

    finalTime = corrected_times(numel(corrected_times));
    
    steadyStateTime = finalTime / 5;

    [~, halfFinalIndex] = closest(corrected_times, steadyStateTime);

    steadyState = velocities(halfFinalIndex:numel(corrected_times));

    expectedSteadyState = mean(steadyState);
    % = K / b
    K_bs(i) = expectedSteadyState;

    expectedB = averageCurrent / expectedSteadyState;

    outputAfterOneTau = (1 - exp(-1)) * expectedSteadyState;

    [~, tauIndex] = closest(velocities, outputAfterOneTau);

    time = corrected_times(tauIndex);
    % = J / b

    J_bs(i) = time;
    
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


%assumption
K_t = 0.124;
b = K_t / average_Kt_b
Fc = average_Fc_b * b
J = average_J_b * b
function [match, index] = closest(values, searchValue) 
    [match, index] = min(abs(values - searchValue));
end

