function frequency_plot(times, values, name)
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