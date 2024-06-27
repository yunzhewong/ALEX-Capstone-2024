close all
clear

[times, currents, velocities, positions] = load_data("data/chirp0to25.csv");

[omega, M, theta] = chirp_identification(times, currents', velocities');

%%
% small w M = 20log10(Kt/J)
% large w: M = 20log10(Kt/J) - 20log10(w)
% breakpoint: w = b/J


[Kt_b, J_b] = method_1(omega, M);
validate_parameters(omega, M, Kt_b, J_b, "Method 1")

[Kt_b, J_b] = method_2(omega, M);
validate_parameters(omega, M, Kt_b, J_b, "Method 2")

[Kt_b, J_b] = method_3(omega, M);
validate_parameters(omega, M, Kt_b, J_b, "Method 3")

function [Kt_b, J_b] = method_1(omega, M)
    SMALL_FREQUENCY = 0.3;
    BREAKPOINT_FREQUENCY = 10.26;

    [~, small_frequency_index] = closest_match(omega, SMALL_FREQUENCY);
    small_w_M = M(small_frequency_index);
    Kt_b = 10 ^ (small_w_M / 20);
    J_b = 1 / BREAKPOINT_FREQUENCY;
end

function [Kt_b, J_b] = method_2(omega, M)
    SMALL_FREQUENCY = 0.3;
    LARGE_FREQUENCY = 120;
    BREAKPOINT_FREQUENCY = 10.26;
    MARGIN = 0.2;
    
    constant_low_frequency = SMALL_FREQUENCY * (1 + MARGIN);
    constant_high_frequency = BREAKPOINT_FREQUENCY * (1 - MARGIN);
    
    sloped_low_frequency = BREAKPOINT_FREQUENCY * (1 + MARGIN);
    sloped_high_frequency = LARGE_FREQUENCY * (1 - MARGIN);
    
    
    [constant_w, constant_m] = extract_range(omega, M, constant_low_frequency, constant_high_frequency);
    constantM = polyfit(constant_w, constant_m, 0);
    
    [sloped_w, sloped_m] = extract_range(omega, M, sloped_low_frequency, sloped_high_frequency);
    sloped_res = polyfit(log10(sloped_w), sloped_m, 1);
    m = sloped_res(1);
    b = sloped_res(2);
    
    
    % figure
    % plot(log10(constant_w), constant_m, 'r-');
    % hold on
    % plot(log10(constant_w), ones(1, numel(constant_w)) * constantM, 'b-')
    % plot(log10(sloped_w), m * log10(sloped_w) + b, 'r-')
    % plot(log10(sloped_w), sloped_m, 'b-')
    % 
    log_breakpoint_freq = (constantM - b) / m;
    breakpoint_freq = 10 ^ log_breakpoint_freq;
    
    Kt_b = 10 ^ (constantM / 20);
    J_b = 1 / breakpoint_freq;
end

function [Kt_b, J_b] = method_3(omega, M)
    SMALL_FREQUENCY = 0.3;
    LARGE_FREQUENCY = 100;

    [~, small_frequency_index] = closest_match(omega, SMALL_FREQUENCY);
    small_w_M = M(small_frequency_index);

    [~, large_frequency_index] = closest_match(omega, LARGE_FREQUENCY);
    large_w_M = M(large_frequency_index);

    m = -20;
    b = -1 * m * log10(LARGE_FREQUENCY) + large_w_M;

    figure
    plot(log10(omega), M)
    hold on
    plot(log10(omega), m * log10(omega) + b)

    log_breakpoint_freq = (small_w_M - b) / m;
    breakpoint_freq = 10 ^ log_breakpoint_freq;

    Kt_b = 10 ^ (small_w_M / 20);
    J_b = 1 / breakpoint_freq;
end

function validate_parameters(omega, M, Kt_b, J_b, name)
    sys = tf(Kt_b, [J_b 1]);
    % 
    % figure
    % bode(sys)
    
    [sysM, ~] = bode(sys, omega);
    sysM = 20*log10(reshape(sysM(1,1, :), [], 1)');
    
    figure
    semilogx(omega, sysM, omega, M);
    title(name)
    
    fprintf("J / b: %f\n", J_b)
    fprintf("Kt / b: %f\n", Kt_b)
end

function [w_range, m_range] = extract_range(freq, mags, low_freq, high_freq)
    [~, low_index] = closest_match(freq, low_freq);
    [~, high_index] = closest_match(freq, high_freq);

    w_range = freq(low_index:high_index);
    m_range = mags(low_index:high_index);
end

function [match, index] = closest_match(values, searchValue) 
    [match, index] = min(abs(values - searchValue));
end


