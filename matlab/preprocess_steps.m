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

    get_fft(corrected_times, currents, "Currents")
    get_fft(corrected_times, velocities, "Velocities")
end

function get_fft(times, values, name)
    sample_time = times(2) - times(1);
    sample_freq = 1 / sample_time;
    total_points = numel(times);
    
    ffted = fft(values);
    
    full_f = sample_freq / total_points * (0:total_points- 1);
 
    P2 = abs(ffted);
    P1 = P2(1:total_points/2+1);
    half_f = full_f(1:total_points / 2 + 1);

    figure
    plot(times, values)
    title(name)

    figure
    plot(half_f, P1)
    title(name)
end

