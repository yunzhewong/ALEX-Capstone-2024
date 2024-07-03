close all

filenames = ["step1A.csv", "step1.1A.csv", "step1.2A.csv", "step1.3A.csv"];

for i=1:numel(filenames)
    fullname = "./data/" + filenames(i);

    data = readmatrix(fullname);

    times = data(:, 1);
    currents = data(:, 2);
    velocities = data(:, 3);

    averageCurrent = mean(currents);

    finalTime = times(numel(times));
    
    halfFinalTime = finalTime / 2;

    [~, halfFinalIndex] = closest(times, halfFinalTime);


    steadyState = velocities(halfFinalIndex:numel(times));

    expectedSteadyState = mean(steadyState);
    % = K / b

    expectedB = averageCurrent / expectedSteadyState;

    outputAfterOneTau = (1 - exp(-1)) * expectedSteadyState;

    [~, tauIndex] = closest(velocities, outputAfterOneTau);

    time = times(tauIndex);
    % = J / b

    expectedJ = time * expectedB;

    

    fprintf("J/b: %f\n", expectedJ / expectedB)

    sys = tf(1/expectedJ, [1 expectedB/expectedJ]);

    simTimes = 0:0.05:finalTime;

 
    [yStep, ~] = step(sys, simTimes);
    figure
    plot(times, velocities)
    hold on
    plot(simTimes, averageCurrent * yStep)



end

function [match, index] = closest(values, searchValue)
    [match, index] = min(abs(values - searchValue));
end