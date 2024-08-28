close all
clear

current_commands = 1.0:0.1:2.2;
count = numel(current_commands);
steady_state_currents = zeros(1, count);
steady_state_velocities = zeros(1, count);

J_bs = zeros(1, count);


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
    
    figure
    hold on
    xline(J_bs(i))
    plot(corrected_times, velocities)
end

function [match, index] = closest(values, searchValue) 
    [match, index] = min(abs(values - searchValue));
end
