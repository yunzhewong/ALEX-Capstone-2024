close all

amperage = 0.46:0.02:0.48;
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

    frequency_plot(corrected_times, currents, "Currents")
    frequency_plot(corrected_times, velocities, "Velocities")
end



